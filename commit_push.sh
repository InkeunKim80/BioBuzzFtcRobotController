#!/bin/bash

set -e

echo "========================================"
echo " FTC Development Commit & Push"
echo "========================================"

# Check that we are inside a Git repository
git rev-parse --is-inside-work-tree > /dev/null 2>&1 || {
    echo "ERROR: Not inside a Git repository."
    exit 1
}

# Get current branch
BRANCH=$(git branch --show-current)

if [ -z "$BRANCH" ]; then
    echo "ERROR: Could not determine current branch."
    exit 1
fi

echo
echo "Current branch: $BRANCH"

# Prevent accidental commits directly to master
if [ "$BRANCH" = "master" ]; then
    echo
    echo "ERROR: You are currently on master."
    echo "Development commits should be made on development."
    echo
    echo "Switch branch with:"
    echo "  git switch development"
    exit 1
fi

# Check for changes
if [ -z "$(git status --porcelain)" ]; then
    echo
    echo "Nothing to commit."
    exit 0
fi

echo
echo "----------------------------------------"
echo "Changes:"
echo "----------------------------------------"
git status --short

echo
read -p "Stage all changes? [y/N]: " CONFIRM

if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

git add -A

echo
echo "----------------------------------------"
echo "Staged changes:"
echo "----------------------------------------"
git status --short

echo
read -p "Enter commit message: " MESSAGE

if [ -z "$MESSAGE" ]; then
    echo "ERROR: Commit message cannot be empty."
    exit 1
fi

echo
echo "Checking origin for remote changes..."
git fetch origin

# Check whether remote branch exists
if git show-ref --verify --quiet "refs/remotes/origin/$BRANCH"; then

    LOCAL=$(git rev-parse "$BRANCH")
    REMOTE=$(git rev-parse "origin/$BRANCH")
    BASE=$(git merge-base "$BRANCH" "origin/$BRANCH")

    if [ "$LOCAL" = "$REMOTE" ]; then
        echo "Local and origin/$BRANCH are synchronized."

    elif [ "$LOCAL" = "$BASE" ]; then
        echo
        echo "ERROR: origin/$BRANCH has commits that are not in your local branch."
        echo "Pull/reconcile remote changes before committing."
        exit 1

    elif [ "$REMOTE" != "$BASE" ]; then
        echo
        echo "ERROR: Local and origin/$BRANCH have diverged."
        echo "Resolve the branch differences before committing."
        exit 1
    fi
fi

echo
echo "Creating commit..."
git commit -m "$MESSAGE"

echo
echo "Pushing to origin/$BRANCH..."
git push -u origin "$BRANCH"

echo
echo "========================================"
echo " Commit & Push completed"
echo " Branch : $BRANCH"
echo " Commit : $(git rev-parse --short HEAD)"
echo " Message: $MESSAGE"
echo "========================================"
