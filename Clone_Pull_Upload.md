# Phase 1: Local Development (On Your Desktop)

First, you manage all your code on your main computer. This is where you make changes and prepare them to be sent to the other devices.

## 1. Initialize the Repository

Go to the folder containing your code and initialize it as a Git repository. This creates a hidden .git folder that tracks all your changes.
```Bash
git init
```
## 2. Add and Commit Your Files

Add your files to the repository's staging area and then "commit" them. A commit is a snapshot of your code at a specific point in time, with a message describing the changes.
```Bash
git add .
git commit -m "Initial commit of recording scripts"
```
The git add . command adds all new and modified files in the current directory.

# Phase 2: Remote Management (Connecting to a Remote Service)

You need a remote service like GitHub, GitLab, or Bitbucket to store the master copy of your code.

## 1. Create a Remote Repository

Go to your chosen service and create a new, empty repository. It will provide you with a URL (e.g., https://github.com/your-username/your-repo.git).

## 2. Link Your Local and Remote Repositories

Connect your local repository to the remote one using the URL.
```Bash
git remote add origin <your-remote-repository-url>
```
## 3. Push Your Code

Now, "push" your local changes to the remote repository. This makes your master copy available in the cloud.
```Bash
git push -u origin main
```
This command sends all your committed code to the main branch of your repository.

# Phase 3: Deployment (On Each Raspberry Pi)

On each of your 5 Raspberry Pi units, you will not push code. Instead, you will pull the latest changes from the central remote repository.

## 1. Clone the Repository

The first time, you will clone the repository. This creates a local copy on the Raspberry Pi.
```Bash
cd /home/pi/
git clone <your-remote-repository-url>
```
### (Note:- Ignore if the repo is already cloned)

## 2.Create a List of Your Raspberry Pis

On your central computer (your desktop), create a simple text file that contains the IP address of each Raspberry Pi on a new line. Let's call this file raspi_list.txt.

Example raspi_list.txt content:
```txt
192.168.1.101
192.168.1.102
192.168.1.103
192.168.1.104
# Add all your Raspberry Pi IP addresses here
```
##  3.Create a Deployment Script

Create a single shell script that will read from this list and automatically log in to each Raspberry Pi to run the git pull command. Let's call this script deploy_pis.sh.
```Bash
sudo nano deploy_pis.sh
```
Paste the following content into the file:
```Bash
#!/bin/bash

# Define the location of the IP list
IP_LIST="raspi_list.txt"

# Check if the list file exists
if [ ! -f "$IP_LIST" ]; then
    echo "Error: IP list file not found!"
    exit 1
fi

# Loop through each IP address in the file
while read -r IP; do
    # Skip empty lines and comments
    if [[ -z "$IP" || "$IP" =~ ^# ]]; then
        continue
    fi
    
    echo "--- Deploying to Raspberry Pi at $IP ---"
    
    # Use SSH to connect and run the git pull command
    ssh pi@"$IP" 'cd /home/pi/<your-project-folder> && git checkout main && git pull'
    
    # You can add more commands here if needed, for example:
    # ssh pi@"$IP" 'g++ /home/pi/your_cpp_file.cpp -o /home/pi/your_program'

    echo "--- Deployment to $IP complete ---"
done < "$IP_LIST"
```
Make the script executable:
```Bash
chmod +x deploy_pis.sh
```
## 3.Run the Script

Now, whenever you want to deploy your latest code to all n of your Raspberry Pis, you just need to run this single script from your desktop.
```Bash
./deploy_pis.sh
```
This will automatically log in to each Raspberry Pi in the list and pull the latest changes from your Git repository.
