{
    "title": "Web Server"
}
# Web Server

## Web Apps

| App | Links |
|-|-|
| [![Foxglove](foxglove/favicon.ico)](foxglove/nav2) | [**Foxglove Studio**](foxglove/nav2) |
| [![Gzweb](gzweb/favicon.ico)](gzweb) | [**Gzweb Client**](gzweb) |
| [![Glances](https://nicolargo.github.io/glances/favicon.ico)](glances) | [**System Monitor**](glances) |

## External Resources

| Site | Links |
|-|-|
[![Nav2](https://navigation.ros.org/_static/nav2_48x48.png)](https://navigation.ros.org) | [Nav2 Documentation](https://navigation.ros.org)

## Server Info

|Key | Value |
|-|-|
| Host | `{{.Host}}` |
| Remote IP | `{{placeholder "http.request.remote.host"}}` |
| Date | `{{now}}` |

### Server Diagnostics

<details>
<summary>Websocket Debug</summary>

|Key | Value |
|-|-|
| `header.X-Forwarded-Host` | `{{placeholder "header.X-Forwarded-Host"}}` |
| `header.X-Forwarded-Scheme` | `{{placeholder "header.X-Forwarded-Scheme"}}` |
| `http.request.hostport` | `{{placeholder "http.request.hostport"}}` |
| `http.request.scheme` | `{{placeholder "http.request.scheme"}}` |
| `http.vars.WsHost` | `{{placeholder "http.vars.WsHost"}}` |
| `http.vars.WsScheme` | `{{placeholder "http.vars.WsScheme"}}` |

</details>
