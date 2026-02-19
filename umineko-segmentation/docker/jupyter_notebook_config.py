c.NotebookApp.terminado_settings = {
    "shell_command": ["/bin/bash"],
    "shell_args": ["-l"]
}

# Browser and IP settings
c.NotebookApp.ip = "0.0.0.0"
c.NotebookApp.open_browser = False
c.NotebookApp.allow_root = True

# Security settings (Recommended for public release)
# By default, Jupyter uses a token. 
# Keeping these commented out encourages safer usage.
# c.NotebookApp.token = ""
# c.NotebookApp.password = ""