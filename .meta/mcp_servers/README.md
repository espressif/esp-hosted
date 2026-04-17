# MCP Servers

MCP servers here are **user-invoked only** — never auto-loaded.

All are configured in `~/.claude/settings.json` under `mcpServers`. Enable/disable per session as needed.

---

## espressif-docs

Espressif technical documentation search.

```json
"espressif-docs": {
  "type": "sse",
  "url": "https://mcp.espressif.com/docs"
}
```

---

## espressif-components

ESP-IDF Component Registry search.

```json
"espressif-components": {
  "type": "sse",
  "url": "https://components.espressif.com/mcp"
}
```

---

## espressif-rainmaker

ESP RainMaker cloud platform API.

```json
"espressif-rainmaker": {
  "type": "sse",
  "url": "https://mcp.rainmaker.espressif.com/api/mcp"
}
```

---

## espressif-gitlab

Espressif internal GitLab — code search, issues, MRs for `gitlab.espressif.cn:6688`.

```json
"espressif-gitlab": {
  "type": "http",
  "url": "https://gitlab.espressif.cn:6688/api/v4/mcp",
  "env": {
    "GITLAB_PERSONAL_ACCESS_TOKEN": "XdH47BzzTg9wRHidozmX"
  }
}
```

---

## athena

Local Athena knowledge base server. Requires the athena-python venv to be set up at `/Users/yogesh/code/athena-python/.venv`.

```json
"athena": {
  "command": "/Users/yogesh/code/athena-python/.venv/bin/athena",
  "args": ["mcp-server", "--enable-write"],
  "env": {
    "ATHENA_BASE_URL": "https://athena.espressif.cn:6565",
    "ATHENA_LOGIN": "your-username",
    "ATHENA_PWD": "your-passwd"
  }
}
```

**Setup**: `cd /Users/yogesh/code/athena-python && uv sync` (or `pip install -e .` in venv).

> Fill in `ATHENA_LOGIN` and `ATHENA_PWD` in `~/.claude/settings.json` before use.

---

## Notes

- Remote servers use `type: "sse"` — if connection fails, try `type: "http"` (newer streamable HTTP transport)
- Athena credentials are stored in plaintext in `settings.json` — do not commit that file
