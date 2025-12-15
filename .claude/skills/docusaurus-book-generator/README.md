# Docusaurus Book Generator

A skill that automates the setup of a Docusaurus-based book website with Tailwind CSS, TypeScript, and GitHub Pages deployment.

## Features

- Creates a complete Docusaurus v3 project with TypeScript and Classic theme
- Configures Tailwind CSS with PostCSS
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
2. Installs and configures Tailwind CSS with PostCSS
3. Updates the Docusaurus configuration with your book's title, tagline, and metadata
4. Removes all default Docusaurus content (tutorials, blog posts)
5. Updates the homepage with your book's information
6. Sets up project structure ready for GitHub Pages deployment (workflow setup handled separately with docusaurus-deployer skill)
7. Sets up a basic sidebar structure for your book
8. Updates the intro page with your book's information

## Output

After running the script, you'll have:

- A fully configured Docusaurus project in the specified output directory
- Tailwind CSS configured and ready to use
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