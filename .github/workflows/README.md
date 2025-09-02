# Instructions on how to trigger Dockerhub.yml github workflow manually for a specific branch:

### 1. From GitHub UI
1. Go to the repository in GitHub.
2. Navigate to **Actions** → select the `DockerHub` workflow.
3. Click **Run workflow** (top-right).
4. Choose the desired branch in the dropdown and confirm.

### 2. From Command Line (API)
You can also trigger the workflow using the GitHub API.  
Replace `YOUR_PERSONAL_ACCESS_TOKEN` with a token that has `workflow` scope, and update `ref` with the branch name.

```bash
curl -X POST \
  -H "Authorization: Bearer YOUR_PERSONAL_ACCESS_TOKEN" \
  -H "Accept: application/vnd.github+json" \
  https://api.github.com/repos/usdot-fhwa-stol/carma-platform/actions/workflows/dockerhub.yml/dispatches \
  -d '{
    "ref": "develop"

  }'