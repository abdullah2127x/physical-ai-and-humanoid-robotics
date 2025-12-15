# Docusaurus Book Generator Command

## Command: `/create-book`

### Description
Creates a new Docusaurus-based book website with Tailwind CSS, TypeScript, and GitHub Pages deployment configured.

### Usage
```
/create-book --title "Your Book Title" [--tagline "Your tagline"] [--description "Your description"] [--github-org "your-org"] [--github-repo "your-repo"] [--output-dir "book-source"]
```

### Parameters
- `--title` (required): The main title of your book
- `--tagline` (optional): Custom tagline (auto-generated if not provided)
- `--description` (optional): Custom description (auto-generated if not provided)
- `--github-org` (optional): GitHub organization for deployment
- `--github-repo` (optional): GitHub repository for deployment
- `--output-dir` (optional): Output directory name (default: "book-source")

### Example
```
/create-book --title "Physical AI & Humanoid Robotics" --tagline "Bridging Digital Minds to Physical Bodies" --github-org "myorg" --github-repo "mybook"
```