#!/bin/bash

set -e

echo "========================================"
echo " Syncing FTC repository with upstream"
echo "========================================"

# Make sure we are inside a Git repository
git rev-parse --is-inside-work-tree > /dev/null 2>&1 || {
    echo "ERROR: Not inside a Git repository."
    exit 1
}

# Make sure working tree is clean
if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: Working tree is not clean."
    echo "Commit or stash your changes before syncing."
    git status --short
    exit 1
fi

# Make sure required remotes exist
git remote get-url origin > /dev/null 2>&1 || {
    echo "ERROR: origin remote does not exist."
    exit 1
}

git remote get-url upstream > /dev/null 2>&1 || {
    echo "ERROR: upstream remote does not exist."
    exit 1
}

echo
echo "[1/6] Fetching upstream..."
git fetch upstream

echo
echo "[2/6] Switching to master..."
git switch master

echo
echo "[3/6] Merging upstream/master into master..."
git merge upstream/master

echo
echo "[4/6] Pushing master to origin..."
git push origin master

echo
echo "[5/6] Switching to development..."
git switch development

echo
echo "[6/6] Merging master into development..."
git merge master

echo
echo "Pushing development to origin..."
git push origin development

echo
echo "========================================"
echo " Sync completed successfully"
echo " Current branch: $(git branch --show-current)"
echo "========================================"
