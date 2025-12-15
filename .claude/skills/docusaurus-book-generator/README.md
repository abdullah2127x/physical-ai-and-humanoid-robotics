# Docusaurus Book Generator

A skill that automates the setup of a Docusaurus-based book website with Tailwind CSS v4, TypeScript, and GitHub Pages deployment.

## Features

- Creates a complete Docusaurus v3 project with TypeScript and Classic theme
- Configures Tailwind CSS v4 with `@tailwindcss/postcss` plugin
- Sets up custom title, tagline, and metadata based on provided parameters
- Removes all default Docusaurus content (tutorials, blog posts)
- Sets up project structure ready for GitHub Pages deployment (workflow setup handled separately with docusaurus-deployer skill)
- Creates a basic book structure ready for content addition

## Usage

Run the setup script with your book's information:

```bash
node .claude/skills/docusaurus-book-generator/scripts/setup-book.ts --title "Your Book Title" --tagline "Your custom tagline" --description "Your book description" --github-org "your-github-org" --github-repo "your-repo-name"
```

### Required Parameters

- `--title`: The main title of your book (required)

### Optional Parameters

- `--tagline`: Custom tagline for your book (auto-generated if not provided)
- `--description`: Custom description for your book (auto-generated if not provided)
- `--github-org`: GitHub organization name for deployment
- `--github-repo`: GitHub repository name for deployment
- `--output-dir`: Directory name for the website (default: "book-source")

### Example Usage

```bash
# Basic setup with just a title
node .claude/skills/docusaurus-book-generator/scripts/setup-book.ts --title "My Awesome Book"

# Complete setup with all parameters
node .claude/skills/docusaurus-book-generator/scripts/setup-book.ts --title "Physical AI & Humanoid Robotics" --tagline "Bridging Digital Minds to Physical Bodies" --description "A comprehensive guide to building intelligent robotic systems" --github-org "myorg" --github-repo "my-book-repo"
```

## What the Skill Does

1. Creates a new Docusaurus v3 project with TypeScript and Classic theme
2. Installs Tailwind CSS v4 with `@tailwindcss/postcss` plugin
3. Creates `postcss.config.js` with correct v4 configuration
4. Creates `tailwind.config.js` with content paths and preflight disabled
5. Updates `src/css/custom.css` with `@import "tailwindcss";` syntax
6. Updates the Docusaurus configuration with your book's title, tagline, and metadata
7. Removes all default Docusaurus content (tutorials, blog posts)
8. Updates the homepage with your book's information
9. Sets up project structure ready for GitHub Pages deployment
10. Sets up a basic sidebar structure for your book
11. Updates the intro page with your book's information

## Tailwind CSS v4 Configuration

This skill uses Tailwind CSS v4, which has different configuration than v3:

### postcss.config.js
```javascript
module.exports = {
  plugins: {
    '@tailwindcss/postcss': {},  // NOT 'tailwindcss'
    autoprefixer: {},
  },
};
```

### tailwind.config.js
```javascript
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
  ],
  theme: { extend: {} },
  plugins: [],
  corePlugins: {
    preflight: false,  // Avoid conflicts with Docusaurus/Infima
  },
};
```

### src/css/custom.css
```css
@import "tailwindcss";  /* NOT @tailwind base; @tailwind components; @tailwind utilities; */

/* Your custom styles... */
```

## Output

After running the script, you'll have:

- A fully configured Docusaurus project in the specified output directory
- Tailwind CSS v4 configured and ready to use
- All default content removed
- Project structure ready for GitHub Pages deployment (use docusaurus-deployer skill for workflow setup)
- A basic book structure ready for content addition
- Proper metadata and SEO configuration

## Next Steps

1. Navigate to your project directory: `cd book-source` (or your specified output directory)
2. Start the development server: `npm run start`
3. Add your book content to the `docs/` directory
4. Update `sidebars.ts` to reflect your book's structure
5. Push to GitHub to trigger the automated deployment workflow (if configured)

## Troubleshooting

### PostCSS Plugin Error
**Error**: `It looks like you're trying to use 'tailwindcss' directly as a PostCSS plugin`
**Solution**: Ensure `@tailwindcss/postcss` is installed and used in postcss.config.js

### CSS Import Error
**Error**: Unknown at-rule `@tailwind`
**Solution**: Use `@import "tailwindcss";` instead of old `@tailwind` directives

### Sidebar Empty Category Error
**Error**: `Sidebar category has neither any subitem nor a link`
**Solution**: Don't create empty sidebar categories - add items or remove the category
