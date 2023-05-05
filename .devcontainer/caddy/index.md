{
    "title": "Web Server",
    "sitename": "Nav2 Dev Container"
}
# Nav2 Dev Container Web Server

## Web Apps

- [Foxglove Studio](foxglove)
  - [Foxglove Studio](foxglove/nav2)
- [Gzweb Client](gzweb)

[Nav2 Documentation](https://navigation.ros.org)

## Status Info

### Server Placeholders

|Key | Value |
|-|-|
| Host | `{{.Host}}` |
| Remote IP | `{{placeholder "http.request.remote.host"}}` |
| Date | `{{now}}` |

### Header Placeholders

|Key | Value |
|-|-|
| `header.X-Forwarded-Host` | `{{placeholder "header.X-Forwarded-Host"}}` |
| `header.X-Forwarded-Scheme` | `{{placeholder "header.X-Forwarded-Scheme"}}` |
| `http.request.hostport` | `{{placeholder "http.request.hostport"}}` |
| `http.request.scheme` | `{{placeholder "http.request.scheme"}}` |
| `http.vars.WsHost` | `{{placeholder "http.vars.WsHost"}}` |
| `http.vars.WsScheme` | `{{placeholder "http.vars.WsScheme"}}` |
