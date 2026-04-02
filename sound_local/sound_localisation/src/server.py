"""
FastAPI server for live sound localization dashboard.
Provides WebSocket endpoint for real-time DOA updates.
"""
import asyncio
import json
import numpy as np
from typing import Set, Optional
from concurrent.futures import ThreadPoolExecutor
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

from src.config import AppConfig
from src.audio import record_multichannel
from src.doa import build_mic_positions, estimate_doa_az_el, estimate_position_from_tdoa


app = FastAPI(title="Sound Localization Dashboard")

# Store active WebSocket connections
active_connections: Set[WebSocket] = set()

# Global config
config = AppConfig()
mic_positions = build_mic_positions(
    radius_m=config.geometry.radius_m,
    angles_deg=config.geometry.mic_angles_deg
)

# Processing state
processing_active = False
processing_task: Optional[asyncio.Task] = None
executor = ThreadPoolExecutor(max_workers=2)


async def broadcast_update(data: dict):
    """Broadcast DOA update to all connected clients."""
    message = json.dumps(data)
    disconnected = set()
    for connection in active_connections:
        try:
            await connection.send_text(message)
        except Exception:
            disconnected.add(connection)
    active_connections.difference_update(disconnected)


def process_audio_chunk():
    """Process a single audio chunk (runs in thread pool)."""
    try:
        # Record audio chunk (blocking call)
        signals = record_multichannel(
            samplerate=config.audio.samplerate,
            duration_s=config.audio.record_seconds,
            dtype=config.audio.dtype,
            channels_to_use=config.audio.channels_to_use,
            device_query=config.audio.device_query,
            requested_channels=config.audio.requested_channels,
            blocksize=config.audio.blocksize,
        )
        
        # Estimate DOA with confidence
        az_deg, el_deg, confidence = estimate_doa_az_el(
            signals=signals,
            fs=config.audio.samplerate,
            mic_positions_m=mic_positions,
            speed_of_sound=config.geometry.speed_of_sound,
            min_correlation_quality=config.detection.min_correlation_quality,
            min_rms_energy=config.detection.min_rms_energy,
            enable_debug=config.detection.enable_debug,
        )
        
        # Always return DOA results, but mark detection status based on confidence
        # This allows dashboard to show markers even for low-confidence detections
        detected = confidence >= config.detection.min_confidence_threshold or config.detection.always_broadcast
        
        if not detected and config.detection.enable_debug:
            print(f"[DEBUG] Confidence {confidence:.3f} below threshold {config.detection.min_confidence_threshold:.3f}, but always_broadcast=True")
        
        # Estimate 3D position and distance
        position_3d = estimate_position_from_tdoa(
            signals=signals,
            fs=config.audio.samplerate,
            mic_positions_m=mic_positions,
            speed_of_sound=config.geometry.speed_of_sound,
            min_correlation_quality=config.detection.min_correlation_quality,
        )
        
        if position_3d is not None:
            distance_m = np.linalg.norm(position_3d)
            x, y, z = position_3d
        else:
            distance_m = None
            x, y, z = None, None, None
        
        # Always return DOA values (azimuth/elevation are always computed)
        # The 'detected' flag indicates if confidence is above threshold
        return {
            "azimuth_deg": float(az_deg) if not np.isnan(az_deg) else 0.0,
            "elevation_deg": float(el_deg) if not np.isnan(el_deg) else 0.0,
            "distance_m": float(distance_m) if distance_m is not None and not np.isnan(distance_m) else None,
            "confidence": float(confidence),
            "position_3d": {
                "x": float(x) if x is not None and not np.isnan(x) else None,
                "y": float(y) if y is not None and not np.isnan(y) else None,
                "z": float(z) if z is not None and not np.isnan(z) else None,
            },
            "detected": detected,
        }
    except Exception as e:
        print(f"Error processing audio: {e}")
        import traceback
        traceback.print_exc()
        return None


async def continuous_audio_processing():
    """Continuously process audio and broadcast DOA updates."""
    global processing_active
    
    print("Starting continuous audio processing...")
    processing_active = True
    
    try:
        while processing_active:
            # Run blocking audio processing in thread pool
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(executor, process_audio_chunk)
            
            if result is None:
                await asyncio.sleep(0.5)
                continue
            
            # Always broadcast DOA updates (let dashboard filter by confidence)
            # This ensures markers are always shown when there's any signal
            if result and (result.get("detected", False) or config.detection.always_broadcast):
                # Prepare update data
                update_data = {
                    "type": "doa_update",
                    **result,
                    "mic_positions": [
                        {"x": float(mic_positions[i, 0]), "y": float(mic_positions[i, 1]), "z": float(mic_positions[i, 2])}
                        for i in range(4)
                    ],
                    "timestamp": asyncio.get_event_loop().time(),
                }
                
                # Broadcast to all connected clients
                await broadcast_update(update_data)
                if config.detection.enable_debug:
                    print(f"[DEBUG] Broadcasted DOA update: az={result.get('azimuth_deg')}, conf={result.get('confidence', 0):.3f}")
            
            # Small delay to prevent overwhelming the system
            await asyncio.sleep(0.1)
            
    except asyncio.CancelledError:
        print("Audio processing cancelled.")
    except Exception as e:
        print(f"Error in audio processing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        processing_active = False
        print("Audio processing stopped.")


@app.get("/", response_class=HTMLResponse)
async def get_dashboard():
    """Serve the dashboard HTML page."""
    import os
    html_path = os.path.join(os.path.dirname(__file__), "static", "dashboard.html")
    with open(html_path, "r") as f:
        return HTMLResponse(content=f.read())


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time DOA updates."""
    await websocket.accept()
    active_connections.add(websocket)
    print(f"Client connected. Total connections: {len(active_connections)}")
    
    # Auto-start processing if not already running
    global processing_task, processing_active
    if not processing_active and processing_task is None:
        processing_task = asyncio.create_task(continuous_audio_processing())
    
    # Send initial mic positions
    await websocket.send_json({
        "type": "init",
        "mic_positions": [
            {"x": float(mic_positions[i, 0]), "y": float(mic_positions[i, 1]), "z": float(mic_positions[i, 2])}
            for i in range(4)
        ],
    })
    
    try:
        while True:
            # Wait for client messages (could be used for control commands)
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                if message.get("type") == "start_processing" and not processing_active:
                    processing_task = asyncio.create_task(continuous_audio_processing())
                elif message.get("type") == "stop_processing":
                    processing_active = False
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        active_connections.discard(websocket)
        print(f"Client disconnected. Total connections: {len(active_connections)}")
        # Stop processing if no clients connected
        if len(active_connections) == 0:
            processing_active = False


@app.on_event("startup")
async def startup_event():
    """Initialize on server startup."""
    print("Sound Localization Dashboard server starting...")
    print("Open http://localhost:8000 in your browser")


@app.on_event("shutdown")
async def shutdown_event():
    """Stop audio processing on server shutdown."""
    global processing_active
    processing_active = False
    if processing_task:
        processing_task.cancel()
        try:
            await processing_task
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

