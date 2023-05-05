{
    "title": "Web Server"
}
# Web Server

## Web Apps

| Apps | Links |
|-|-|
| [![Foxglove](foxglove/favicon.ico)](foxglove/nav2) | [**Foxglove Studio**](foxglove/nav2) |
| [![Gzweb](gzweb/favicon.ico)](gzweb) | [**Gzweb Client**](gzweb) |

## Resources

| Resources | Links |
|-|-|
[![Nav2](https://navigation.ros.org/_static/nav2_48x48.png)](https://navigation.ros.org) | [Nav2 Documentation](https://navigation.ros.org)

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
| `http.vars.ws.host` | `{{placeholder "http.vars.ws.host"}}` |
| `http.vars.ws.scheme` | `{{placeholder "http.vars.ws.scheme"}}` |
