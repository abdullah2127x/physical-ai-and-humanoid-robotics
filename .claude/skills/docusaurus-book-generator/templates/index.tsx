import clsx from 'clsx';
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
            {{BOOK_TITLE}}
          </Heading>
          <p className="hero__subtitle" aria-label="Tagline: {{BOOK_TAGLINE}}">
            {{BOOK_TAGLINE}}
          </p>
          <div className={styles.buttons} role="group" aria-label="Primary navigation buttons">
            <Link
              className="button button--secondary button--lg margin-right--md"
              to="/docs/intro"
              aria-label="Start learning with Module 1">
              Start Learning
            </Link>
            <Link
              className="button button--outline button--lg"
              to="#"
              aria-label="Additional resources">
              Resources
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
      title={`{{BOOK_TITLE}}`}
      description="{{BOOK_DESCRIPTION}}">
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
                  <p id="feature1-description">Learn the fundamental concepts of {{BOOK_TITLE}}</p>
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