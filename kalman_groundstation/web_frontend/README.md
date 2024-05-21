1. Usable scripts can be found in `package.json` > `"scripts"`
2. Install `eslint` plugin for vscode.
3. It is also recommended to create `settings.json` file in `.vscode` directory containing:
javascript```
{
    "editor.codeActionsOnSave": {
        "source.fixAll.eslint": true
    },
    "eslint.validate": [
        "javascript", 
        "typescript", 
        "javascriptreact", 
        "typescriptreact"
    ],
}
```
