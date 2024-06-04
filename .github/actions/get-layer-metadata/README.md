# get-layer-metadata

GitHub Action to get layer metadata from Docker Buildx Bake output result.

## Usage

```yaml
jobs:
  get-layer-metadata:
    runs-on: ubuntu-latest
    steps:
      - name: Check out repository
        uses: actions/checkout@v4

      - name: Build and load
        id: docker_bake_load
        uses: docker/bake-action@v4
        with:
          push: false
          load: true
      - name: Get Layer Metadata Locally
        id: get_layer_metadata_local
        uses: ros-navigation/navigation2/.github/actions/get-layer-metadata@main
        with:
          metadata: ${{ steps.docker_bake_load.outputs.metadata }}
          load: true

      - name: Build and push
        id: docker_bake_push
        uses: docker/bake-action@v4
        with:
          push: true
          load: false
      - name: Get Layer Metadata Remotely
        id: get_layer_metadata_remote
        uses: ros-navigation/navigation2/.github/actions/get-layer-metadata@main
        with:
          metadata: ${{ steps.docker_bake_push.outputs.metadata }}
          load: false
    
      - name: Layer metadata
        id: layer_metadata
        run: |
          local_layer_digest="${{ fromJSON(steps.get_layer_metadata_local.outputs.metadata)['<target_name_here>']['layer.digest'] }}"
          remote_layer_digest="${{ fromJSON(steps.get_layer_metadata_remote.outputs.metadata)['<target_name_here>']['layer.digest'] }}"

```

## Inputs

| Name | Description |
| --- | --- |
| metadata | Build result metadata |
| load | Load is a shorthand to use local registry |

## Outputs

| Name | Description |
| --- | --- |
| metadata | Layer result metadata |