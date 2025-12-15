#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { execSync } from 'child_process';

/**
 * Docusaurus Book Generator Script
 *
 * This script automates the setup of a Docusaurus book website with:
 * - Custom title, tagline, and metadata
 * - Tailwind CSS v4 configuration (with @tailwindcss/postcss)
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
  console.log('Installing Tailwind CSS v4...');
  const cwd = process.cwd();
  process.chdir(outputDir);

  // Install Tailwind CSS v4 with @tailwindcss/postcss plugin
  execSync('npm install -D tailwindcss @tailwindcss/postcss autoprefixer', { stdio: 'inherit' });

  process.chdir(cwd);
}

function configureTailwind(outputDir: string): void {
  console.log('Configuring Tailwind CSS v4...');

  // Create postcss.config.js with @tailwindcss/postcss (Tailwind v4 requirement)
  const postcssConfigPath = path.join(outputDir, 'postcss.config.js');
  const postcssConfig = `module.exports = {
  plugins: {
    '@tailwindcss/postcss': {},
    autoprefixer: {},
  },
};
`;
  fs.writeFileSync(postcssConfigPath, postcssConfig);

  // Create tailwind.config.js
  const tailwindConfigPath = path.join(outputDir, 'tailwind.config.js');
  const tailwindConfig = `/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
  ],
  theme: {
    extend: {},
  },
  plugins: [],
  corePlugins: {
    preflight: false, // Disable preflight to avoid conflicts with Docusaurus/Infima
  },
};
`;
  fs.writeFileSync(tailwindConfigPath, tailwindConfig);

  // Update src/css/custom.css to use Tailwind v4 import syntax
  const customCssPath = path.join(outputDir, 'src', 'css', 'custom.css');
  if (fs.existsSync(customCssPath)) {
    let customCss = fs.readFileSync(customCssPath, 'utf8');
    // Add Tailwind v4 import at the top (NOT the old @tailwind directives)
    if (!customCss.includes('@import "tailwindcss"')) {
      customCss = '@import "tailwindcss";\n\n' + customCss;
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
sidebar_position: 1
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
    const homepageContent = `import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Core Concepts',
    description: (
      <>
        Learn the fundamental concepts of ${config.title}
      </>
    ),
  },
  {
    title: 'Practical Applications',
    description: (
      <>
        Apply concepts through hands-on exercises and examples
      </>
    ),
  },
  {
    title: 'Advanced Topics',
    description: (
      <>
        Explore advanced topics and current research
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md padding-vert--lg">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Welcome"
      description="${config.description || generateDescription(config.title)}">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
`;
    fs.writeFileSync(homepagePath, homepageContent);
  }

  // Update index.module.css to include features style
  const cssPath = path.join(outputDir, 'src', 'pages', 'index.module.css');
  if (fs.existsSync(cssPath)) {
    let cssContent = fs.readFileSync(cssPath, 'utf8');
    if (!cssContent.includes('.features')) {
      cssContent += `
.features {
  display: flex;
  align-items: center;
  padding: 2rem 0;
  width: 100%;
}
`;
      fs.writeFileSync(cssPath, cssContent);
    }
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

  // Remove default HomepageFeatures component (we create our own in index.tsx)
  const homepageFeaturesPath = path.join(outputDir, 'src', 'components', 'HomepageFeatures');
  if (fs.existsSync(homepageFeaturesPath)) {
    fs.rmSync(homepageFeaturesPath, { recursive: true, force: true });
  }
}


function updateSidebar(outputDir: string): void {
  console.log('Updating sidebar configuration...');

  const sidebarPath = path.join(outputDir, 'sidebars.ts');
  if (fs.existsSync(sidebarPath)) {
    // Simple sidebar that won't cause empty category errors
    const sidebarContent = `import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Book sidebar configuration
 * Add chapters as folders and documents in the docs/ directory.
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    // Add more chapters here as you create content
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

  // Install and configure Tailwind CSS v4
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
