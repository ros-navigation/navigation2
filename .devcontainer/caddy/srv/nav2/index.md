{
    "title": "Nav2 App"
}
## Hosted Apps

Redirects for available web apps:

| App | Links |
|-|-|
| [<img src="/media/icons/foxglove/any_icon_x512.webp" height="64">](/foxglove/autolayout) | [**Foxglove Studio**](/foxglove/autolayout) |
| [<img src="/media/icons/gzweb/any_icon_x512.webp" height="64">](/gzweb/) | [**Gzweb Client**](/gzweb/) |
| [<img src="/media/icons/glances/any_icon_x512.webp" height="64">](/glances/) | [**System Monitor**](/glances/) |
| [<img src="/media/icons/nav2/any_icon_x512.webp" height="64">](/nav2/) | [**Nav2 App**](/nav2/) |

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
<summary>File Browser</summary>

For troubleshooting file server:

- [Assets](/assets/)
  - Assets internal to workspace
- [Media](/media/)
  - Media external to workspace

</details>

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
