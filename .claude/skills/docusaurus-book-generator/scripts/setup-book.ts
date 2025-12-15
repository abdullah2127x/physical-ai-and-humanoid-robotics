#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { execSync } from 'child_process';

/**
 * Docusaurus Book Generator Script
 *
 * This script automates the setup of a Docusaurus book website with:
 * - Custom title, tagline, and metadata
 * - Tailwind CSS configuration
 * - Removal of default content
 * - GitHub Actions workflow for GitHub Pages
 */

interface BookConfig {
  title: string;
  tagline?: string;
  description?: string;
  githubOrg?: string;
  githubRepo?: string;
  outputDir?: string;
}

function generateTagline(title: string): string {
  return `Exploring ${title}`;
}

function generateDescription(title: string): string {
  return `Learn about ${title} - comprehensive guide and documentation`;
}

function generateKeywords(title: string): string {
  return title.toLowerCase().replace(/\s+/g, ', ') + ', documentation, guide, tutorial';
}

function createDocusaurusProject(outputDir: string): void {
  console.log('Creating Docusaurus project...');
  execSync(`npx create-docusaurus@latest ${outputDir} classic --typescript`, { stdio: 'inherit' });
}

function installTailwind(outputDir: string): void {
  console.log('Installing Tailwind CSS...');
  const cwd = process.cwd();
  process.chdir(outputDir);

  execSync('npm install -D tailwindcss postcss autoprefixer', { stdio: 'inherit' });
  execSync('npx tailwindcss init -p', { stdio: 'inherit' });

  process.chdir(cwd);
}

function configureTailwind(outputDir: string): void {
  console.log('Configuring Tailwind CSS...');

  // Update tailwind.config.js
  const tailwindConfigPath = path.join(outputDir, 'tailwind.config.js');
  const tailwindConfig = `/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.{md,mdx}",
    "./blog/**/*.{md,mdx}",
  ],
  theme: {
    extend: {},
  },
  plugins: [],
}
`;
  fs.writeFileSync(tailwindConfigPath, tailwindConfig);

  // Update src/css/custom.css to include Tailwind directives
  const customCssPath = path.join(outputDir, 'src', 'css', 'custom.css');
  if (fs.existsSync(customCssPath)) {
    let customCss = fs.readFileSync(customCssPath, 'utf8');
    // Add Tailwind directives at the top if not already present
    if (!customCss.includes('@tailwind')) {
      customCss = '@tailwind base;\n@tailwind components;\n@tailwind utilities;\n\n' + customCss;
      fs.writeFileSync(customCssPath, customCss);
    }
  }
}

function updateDocusaurusConfig(outputDir: string, config: BookConfig): void {
  console.log('Updating Docusaurus configuration...');

  const docusaurusConfigPath = path.join(outputDir, 'docusaurus.config.ts');

  // Read the existing config
  let configContent = fs.readFileSync(docusaurusConfigPath, 'utf8');

  // Replace title, tagline, and other metadata
  configContent = configContent.replace(
    /title: '.*',/,
    `title: '${config.title}',`
  );

  configContent = configContent.replace(
    /tagline: '.*',/,
    `tagline: '${config.tagline || generateTagline(config.title)}',`
  );

  // Update URL and base URL for GitHub Pages
  if (config.githubOrg && config.githubRepo) {
    configContent = configContent.replace(
      /url: '.*',/,
      `url: 'https://${config.githubOrg}.github.io',`
    );

    configContent = configContent.replace(
      /baseUrl: '.*',/,
      `baseUrl: '/${config.githubRepo}/',`
    );

    // Update organization and project names
    configContent = configContent.replace(
      /organizationName: '.*',/,
      `organizationName: '${config.githubOrg}',`
    );

    configContent = configContent.replace(
      /projectName: '.*',/,
      `projectName: '${config.githubRepo}',`
    );
  }

  // Update navbar title
  configContent = configContent.replace(
    /title: '.*',/,
    `title: '${config.title}',`
  );

  // Update description in metadata
  if (config.description) {
    configContent = configContent.replace(
      /content: 'Learn .* Docusaurus'/,
      `content: '${config.description}'`
    );
  }

  // Disable blog if not needed for a book
  configContent = configContent.replace(
    /blog: \{[^}]*\},/,
    'blog: false, // Disable blog functionality for the book'
  );

  // Update edit URL if GitHub repo is provided
  if (config.githubOrg && config.githubRepo) {
    configContent = configContent.replace(
      /editUrl:\s*'https:\/\/github\.com\/.*',/g,
      `          editUrl:\n            'https://github.com/${config.githubOrg}/${config.githubRepo}/edit/main/${outputDir}/docs/',`
    );
  }

  fs.writeFileSync(docusaurusConfigPath, configContent);
}

function updateIntroPage(outputDir: string, config: BookConfig): void {
  console.log('Updating intro page...');

  const introPath = path.join(outputDir, 'docs', 'intro.md');
  if (fs.existsSync(introPath)) {
    const introContent = `---
title: Introduction
description: Introduction to ${config.title}
---

# Introduction

Welcome to ${config.title}. This introduction provides an overview of the concepts and modules you'll encounter throughout this book.
`;
    fs.writeFileSync(introPath, introContent);
  }
}

function updateHomepage(outputDir: string, config: BookConfig): void {
  console.log('Updating homepage...');

  const homepagePath = path.join(outputDir, 'src', 'pages', 'index.tsx');
  if (fs.existsSync(homepagePath)) {
    const homepageContent = `import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)} role="banner">
      <div className="container">
        <div className="text--center">
          <Heading as="h1" className="hero__title" aria-level="1">
            ${config.title}
          </Heading>
          <p className="hero__subtitle" aria-label="Tagline: ${config.tagline || generateTagline(config.title)}">
            ${config.tagline || generateTagline(config.title)}
          </p>
          <div className={styles.buttons} role="group" aria-label="Primary navigation buttons">
            <Link
              className="button button--secondary button--lg margin-right--md"
              to="/docs/intro"
              aria-label="Start learning with the introduction">
              Start Learning
            </Link>
            <Link
              className="button button--outline button--lg"
              to="/docs/tutorial-basics/create-a-page"
              aria-label="Explore documentation">
              Explore Docs
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={\`Welcome to \${siteConfig.title}\`}
      description="${config.description || generateDescription(config.title)}">
      <HomepageHeader />
      <main>
        <section className={styles.features} aria-labelledby="features-heading">
          <h2 id="features-heading" className="visually-hidden">Key Learning Areas</h2>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--4 padding--md">
                <div className="text--center padding--sm"
                     style={{border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '8px'}}
                     role="region"
                     aria-labelledby="feature1-heading"
                     aria-describedby="feature1-description">
                  <Heading as="h3" id="feature1-heading">Core Concepts</Heading>
                  <p id="feature1-description">Learn the fundamental concepts of ${config.title}</p>
                </div>
              </div>
              <div className="col col--4 padding--md">
                <div className="text--center padding--sm"
                     style={{border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '8px'}}
                     role="region"
                     aria-labelledby="feature2-heading"
                     aria-describedby="feature2-description">
                  <Heading as="h3" id="feature2-heading">Practical Applications</Heading>
                  <p id="feature2-description">Apply concepts through hands-on exercises and examples</p>
                </div>
              </div>
              <div className="col col--4 padding--4">
                <div className="text--center padding--sm"
                     style={{border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '8px'}}
                     role="region"
                     aria-labelledby="feature3-heading"
                     aria-describedby="feature3-description">
                  <Heading as="h3" id="feature3-heading">Advanced Topics</Heading>
                  <p id="feature3-description">Explore advanced topics and current research</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
`;
    fs.writeFileSync(homepagePath, homepageContent);
  }
}

function removeDefaultContent(outputDir: string): void {
  console.log('Removing default content...');

  // Remove tutorial-basics and tutorial-extras directories
  const tutorialBasicsPath = path.join(outputDir, 'docs', 'tutorial-basics');
  const tutorialExtrasPath = path.join(outputDir, 'docs', 'tutorial-extras');

  if (fs.existsSync(tutorialBasicsPath)) {
    fs.rmSync(tutorialBasicsPath, { recursive: true, force: true });
  }

  if (fs.existsSync(tutorialExtrasPath)) {
    fs.rmSync(tutorialExtrasPath, { recursive: true, force: true });
  }

  // Remove blog directory if we're treating this as a book
  const blogPath = path.join(outputDir, 'blog');
  if (fs.existsSync(blogPath)) {
    fs.rmSync(blogPath, { recursive: true, force: true });
  }
}


function updateSidebar(outputDir: string): void {
  console.log('Updating sidebar configuration...');

  const sidebarPath = path.join(outputDir, 'sidebars.ts');
  if (fs.existsSync(sidebarPath)) {
    const sidebarContent = `import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Sidebar for the book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Getting Started',
      items: ['your-first-chapter'],
    },
    // Add more categories as needed for your book
  ],
};

export default sidebars;
`;
    fs.writeFileSync(sidebarPath, sidebarContent);
  }
}

function main(): void {
  // Get command line arguments
  const args = process.argv.slice(2);

  if (args.length === 0) {
    console.error('Usage: node setup-book.js --title "Your Book Title" [--tagline "Your tagline"] [--description "Your description"] [--github-org "your-org"] [--github-repo "your-repo"]');
    process.exit(1);
  }

  const config: BookConfig = { title: '' };

  for (let i = 0; i < args.length; i += 2) {
    const flag = args[i];
    const value = args[i + 1];

    switch (flag) {
      case '--title':
        config.title = value;
        break;
      case '--tagline':
        config.tagline = value;
        break;
      case '--description':
        config.description = value;
        break;
      case '--github-org':
        config.githubOrg = value;
        break;
      case '--github-repo':
        config.githubRepo = value;
        break;
      case '--output-dir':
        config.outputDir = value;
        break;
      default:
        console.error(`Unknown flag: ${flag}`);
        process.exit(1);
    }
  }

  if (!config.title) {
    console.error('Title is required');
    process.exit(1);
  }

  const outputDir = config.outputDir || 'book-source';

  console.log(`Setting up Docusaurus book: ${config.title}`);

  // Create the Docusaurus project
  createDocusaurusProject(outputDir);

  // Install and configure Tailwind CSS
  installTailwind(outputDir);
  configureTailwind(outputDir);

  // Update configurations
  updateDocusaurusConfig(outputDir, config);
  updateIntroPage(outputDir, config);
  updateHomepage(outputDir, config);
  updateSidebar(outputDir);

  // Remove default content
  removeDefaultContent(outputDir);

  // GitHub Actions workflow will be set up separately using docusaurus-deployer skill
  console.log('Note: GitHub Actions workflow setup will be handled separately using docusaurus-deployer skill');

  console.log('\nDocusaurus book setup complete!');
  console.log(`Project created in: ${outputDir}`);
  console.log('Next steps:');
  console.log(`1. cd ${outputDir}`);
  console.log('2. npm run start # to start the development server');
  console.log('3. Add your book content to the docs/ directory');
  console.log('4. Update sidebars.ts to reflect your book structure');
}

main();