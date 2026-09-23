#!/bin/bash

set -e

echo "========================================"
echo " Merge development into master"
echo "========================================"

# Make sure we are inside a Git repository
git rev-parse --is-inside-work-tree > /dev/null 2>&1 || {
    echo "ERROR: Not inside a Git repository."
    exit 1
}

# Make sure working tree is clean
if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: Working tree is not clean."
    echo "Commit or stash your changes before merging."
    git status --short
    exit 1
fi

# Check required branches/remotes
git show-ref --verify --quiet refs/heads/development || {
    echo "ERROR: Local development branch does not exist."
    exit 1
}

git show-ref --verify --quiet refs/heads/master || {
    echo "ERROR: Local master branch does not exist."
    exit 1
}

git remote get-url origin > /dev/null 2>&1 || {
    echo "ERROR: origin remote does not exist."
    exit 1
}

echo
echo "[1/6] Fetching latest origin..."
git fetch origin

echo
echo "[2/6] Updating local development..."
git switch development
git pull --ff-only origin development

echo
echo "[3/6] Updating local master..."
git switch master
git pull --ff-only origin master

echo
echo "[4/6] Merging development into master..."
git merge --no-ff development

echo
echo "[5/6] Pushing master to origin..."
git push origin master

echo
echo "[6/6] Switching back to development..."
git switch development

echo
echo "========================================"
echo " Merge completed successfully"
echo " development has been merged into master"
echo " Current branch: $(git branch --show-current)"
echo "========================================"
