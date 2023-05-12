{
    "title": "Web Server"
}
## Hosted Apps

Redirects for available web apps:

| App | Links |
|-|-|
| [![Foxglove](foxglove/favicon.ico)](foxglove/autolayout) | [**Foxglove Studio**](foxglove/autolayout) |
| [![Gzweb](gzweb/favicon.ico)](gzweb) | [**Gzweb Client**](gzweb) |
| [![Glances](https://nicolargo.github.io/glances/favicon.ico)](glances) | [**System Monitor**](glances) |

## External Resources

For more related documentation:

- [Nav2 Documentation](https://navigation.ros.org)
  - [Development Guides](https://navigation.ros.org/development_guides)
    - [Dev Containers](https://navigation.ros.org/development_guides/devcontainer_docs)

## Session Info

Useful information about host server and remote client:

|Key | Value |
|-|-|
| Host | `{{.Host}}` |
| Remote IP | `{{placeholder "http.request.remote.host"}}` |
| Date | `{{now}}` |

### Server Diagnostics

<details>
<summary>Websocket Debug</summary>

For troubleshooting websocket connections:

|Key | Value |
|-|-|
| `header.X-Forwarded-Host` | `{{placeholder "header.X-Forwarded-Host"}}` |
| `header.X-Forwarded-Scheme` | `{{placeholder "header.X-Forwarded-Scheme"}}` |
| `http.request.hostport` | `{{placeholder "http.request.hostport"}}` |
| `http.request.scheme` | `{{placeholder "http.request.scheme"}}` |
| `http.vars.WsHost` | `{{placeholder "http.vars.WsHost"}}` |
| `http.vars.WsScheme` | `{{placeholder "http.vars.WsScheme"}}` |

</details>
