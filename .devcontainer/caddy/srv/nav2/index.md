{
    "title": "Nav2 App"
}
## Progressive Web Apps

| PWAs | Shortcuts |
|-|-|
| [<img src="/media/icons/foxglove/any_icon_x512.webp" height="64">](/foxglove/autoconnect)<br>**Foxglove** | [**Auto Connect**](/foxglove/autoconnect)<br>[Auto Layout](/foxglove/autolayout)<br>[Manual](/foxglove/) |
| [<img src="/media/icons/gzweb/any_icon_x512.webp" height="64">](/gzweb/)<br>**Gzweb** | [**Auto Connect**](/gzweb/) |
| [<img src="/media/icons/glances/any_icon_x512.webp" height="64">](/glances/)<br>**Glances** | [**System Monitor**](/glances/)<br>[Refresh 1sec](/glances/1)<br>[Refresh 10sec](/glances/10) |
| [<img src="/media/icons/nav2/any_icon_x512.webp" height="64">](/nav2/)<br>**Nav2** | [**App Launcher**](/nav2/)<br>[File Browser](/?browse=true) |

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
| `header.X-Forwarded-Host` | `{{placeholder "http.request.header.X-Forwarded-Host"}}` |
| `http.request.hostport` | `{{placeholder "http.request.hostport"}}` |
| `http.vars.ReqHost` | `{{placeholder "http.vars.ReqHost"}}` |

|Key | Value |
|-|-|
| `http.request.scheme` | `{{placeholder "http.request.scheme"}}` |
| `header.X-Forwarded-Scheme` | `{{placeholder "http.request.header.X-Forwarded-Scheme"}}` |
| `header.X-Forwarded-Proto` | `{{placeholder "http.request.header.X-Forwarded-Proto"}}` |
| `http.vars.WsScheme` | `{{placeholder "http.vars.WsScheme"}}` |

</details>
