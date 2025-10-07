# Localized Sound Final

This repository contains a Jupyter Notebook for sound localization using signal processing and/or machine learning techniques.

## Files

- `Localized_Sound_final.ipynb`: The main notebook containing the implementation.
- `README.md`: This file.

## How to Run

1. Open the notebook in Jupyter or Google Colab.
2. Install necessary dependencies if prompted.
3. Follow the cells sequentially to replicate the results.

## Requirements

- Python 3.8+
- Jupyter Notebook
- Libraries: `numpy`, `matplotlib`, `librosa`, `scipy`, etc.

## 🚀 Essential Git Commands for Repository Management

This section provides the core commands required to synchronize your local development environment with the remote repository on GitHub.
### Standard Workflow Cycle

Use these four commands every time you save local work and push it to the cloud.
#### 1. Synchronize (Get Latest Changes)

Before starting work or pushing your changes, always ensure your local repository is up-to-date with the remote branch.
```bash
git pull origin main
```

Purpose: Fetches changes from the remote (origin) and merges them into your local main branch.

#### 2. Stage Changes

Tell Git which modified or new files you want to include in your next saved snapshot (commit).

###### To stage specific files (like the characterization scripts)
```bash
git add frequency_response_test.py live_acoustic_performance_test.py directional_accuracy_guide.md
```

##### OR, to stage all changes in the current directory
```bash
git add .
```
Purpose: Moves files from the working directory into the staging area.

#### 3. Commit (Save Local Snapshot)

Permanently record the staged changes into your local repository history.
```bash
git commit -m "Your descriptive message about the changes."
```
Purpose: Creates a local, stable checkpoint of your work.

#### 4. Push (Share Changes)

Send all your new local commits up to the remote repository on GitHub.
```bash
git push origin main
```
Purpose: Uploads your local commits to the main branch on GitHu.


## ❗ Handling Conflicts (If Pushing Fails)

If git push fails because the remote has changes you don't have (a "rejected" error), you must pull, resolve any conflicts, and then push again.

### Step A: Attempt to merge remote changes
```bash
git pull origin main --no-rebase 
```
### Step B: If conflicts occur (manual step), resolve them, then stage the result
```bash
git add . 
```
### Step C: Complete the merge commit
```bash
git commit -m "Merged remote changes."
```
### Step D: Push the final, merged history
```bash
git push origin main
```


## Author

Amogh Ukkadagatri
