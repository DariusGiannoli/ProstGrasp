# 🧪 phospho documentation

This is the user-facing documentation of the [phospho platform](https://platform.phospho.ai)

- The deployed docs [are available here.](https://docs.phospho.ai/welcome)
- The open source code of the platform [is available here.](https://github.com/phospho-app/phospho)

## Local development

The docs use Mintlify for deployment. Install the [Mintlify CLI](https://www.npmjs.com/package/mintlify) to preview the documentation changes locally. To install, use the following command

Use node version 18. Install with nvm.

```
npm i -g mintlify
```

Run the following command at the root of your documentation (where mint.json is)

```
mintlify dev
```

To learn how to format the pages and what blocks you can use, [check out the Mintlify docs.](https://mintlify.com/docs/so-100/quickstart)

### Troubleshooting

- `mintlify dev` doesn't run - try `npx mintlify dev`
- Mintlify dev isn't running - Run `mintlify install` it'll re-install dependencies.
- Page loads as a 404 - Make sure you are running in a folder with `mint.json`
