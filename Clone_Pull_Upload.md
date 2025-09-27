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
## 2. Pull the Latest Changes

When you make updates on your desktop and push them to the remote repository, you can update each Raspberry Pi with a single command.
```Bash
cd <your-project-directory>
git pull
```
This git pull command will download and apply the latest changes, keeping all your Raspberry Pis perfectly synchronized.
