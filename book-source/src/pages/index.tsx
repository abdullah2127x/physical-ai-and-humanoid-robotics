import { useState, useEffect, useCallback, type ReactNode } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';

import styles from './index.module.css';

// Animated Robot Component with Cursor-Tracking Eyes
function AnimatedRobot() {
  const [eyePosition, setEyePosition] = useState({ x: 0, y: 0 });
  const [isBlinking, setIsBlinking] = useState(false);

  const handleMouseMove = useCallback((e: MouseEvent) => {
    // Get the robot container position
    const robotElement = document.getElementById('robot-container');
    if (!robotElement) return;

    const rect = robotElement.getBoundingClientRect();
    const robotCenterX = rect.left + rect.width / 2;
    const robotCenterY = rect.top + rect.height / 3; // Eyes are in upper third

    // Calculate angle and distance from robot center to cursor
    const deltaX = e.clientX - robotCenterX;
    const deltaY = e.clientY - robotCenterY;

    // Limit eye movement range
    const maxMove = 8;
    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    const normalizedDistance = Math.min(distance / 300, 1);

    const moveX = (deltaX / (distance || 1)) * maxMove * normalizedDistance;
    const moveY = (deltaY / (distance || 1)) * maxMove * normalizedDistance;

    setEyePosition({ x: moveX, y: moveY });
  }, []);

  useEffect(() => {
    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, [handleMouseMove]);

  // Random blinking effect
  useEffect(() => {
    const blinkInterval = setInterval(() => {
      if (Math.random() > 0.7) {
        setIsBlinking(true);
        setTimeout(() => setIsBlinking(false), 150);
      }
    }, 2000);

    return () => clearInterval(blinkInterval);
  }, []);

  return (
    <div id="robot-container" className={styles.robotContainer}>
      <svg
        viewBox="0 0 200 280"
        className={styles.robotSvg}
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Definitions for gradients and filters */}
        <defs>
          {/* Body gradient */}
          <linearGradient id="bodyGradient" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#2a2a3e" />
            <stop offset="50%" stopColor="#1a1a2e" />
            <stop offset="100%" stopColor="#0f0f1a" />
          </linearGradient>

          {/* Accent gradient */}
          <linearGradient id="accentGradient" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stopColor="#00ff88" />
            <stop offset="100%" stopColor="#00aa55" />
          </linearGradient>

          {/* Eye glow */}
          <radialGradient id="eyeGlow" cx="50%" cy="50%" r="50%">
            <stop offset="0%" stopColor="#00ffff" stopOpacity="1" />
            <stop offset="70%" stopColor="#00aaff" stopOpacity="0.8" />
            <stop offset="100%" stopColor="#0066ff" stopOpacity="0" />
          </radialGradient>

          {/* Core glow */}
          <radialGradient id="coreGlow" cx="50%" cy="50%" r="50%">
            <stop offset="0%" stopColor="#00ff88" stopOpacity="1" />
            <stop offset="60%" stopColor="#00ff88" stopOpacity="0.5" />
            <stop offset="100%" stopColor="#00ff88" stopOpacity="0" />
          </radialGradient>

          {/* Glow filter */}
          <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
            <feGaussianBlur stdDeviation="3" result="coloredBlur" />
            <feMerge>
              <feMergeNode in="coloredBlur" />
              <feMergeNode in="SourceGraphic" />
            </feMerge>
          </filter>

          {/* Strong glow filter */}
          <filter id="strongGlow" x="-100%" y="-100%" width="300%" height="300%">
            <feGaussianBlur stdDeviation="6" result="coloredBlur" />
            <feMerge>
              <feMergeNode in="coloredBlur" />
              <feMergeNode in="coloredBlur" />
              <feMergeNode in="SourceGraphic" />
            </feMerge>
          </filter>
        </defs>

        {/* Antenna */}
        <g className={styles.antenna}>
          <line x1="100" y1="25" x2="100" y2="5" stroke="#00ff88" strokeWidth="2" />
          <circle cx="100" cy="5" r="4" fill="#00ff88" filter="url(#strongGlow)" className={styles.antennaLight} />
        </g>

        {/* Head */}
        <g className={styles.robotHead}>
          {/* Head base */}
          <rect x="55" y="25" width="90" height="70" rx="15" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1.5" />

          {/* Visor/Face plate */}
          <rect x="62" y="35" width="76" height="45" rx="10" fill="#0a0a15" stroke="#00aaff" strokeWidth="1" opacity="0.9" />

          {/* Eyes container */}
          <g className={styles.eyes}>
            {/* Left eye socket */}
            <ellipse cx="80" cy="57" rx="14" ry={isBlinking ? 2 : 12} fill="#001122" className={styles.eyeSocket} />
            {/* Left eye */}
            <ellipse
              cx={80 + eyePosition.x}
              cy={57 + eyePosition.y}
              rx="8"
              ry={isBlinking ? 1 : 8}
              fill="url(#eyeGlow)"
              filter="url(#glow)"
              className={styles.eye}
            />
            {/* Left pupil */}
            <circle
              cx={80 + eyePosition.x * 1.2}
              cy={57 + eyePosition.y * 1.2}
              r={isBlinking ? 0 : 3}
              fill="#ffffff"
              className={styles.pupil}
            />

            {/* Right eye socket */}
            <ellipse cx="120" cy="57" rx="14" ry={isBlinking ? 2 : 12} fill="#001122" className={styles.eyeSocket} />
            {/* Right eye */}
            <ellipse
              cx={120 + eyePosition.x}
              cy={57 + eyePosition.y}
              rx="8"
              ry={isBlinking ? 1 : 8}
              fill="url(#eyeGlow)"
              filter="url(#glow)"
              className={styles.eye}
            />
            {/* Right pupil */}
            <circle
              cx={120 + eyePosition.x * 1.2}
              cy={57 + eyePosition.y * 1.2}
              r={isBlinking ? 0 : 3}
              fill="#ffffff"
              className={styles.pupil}
            />
          </g>

          {/* Mouth/Speaker grille */}
          <g className={styles.mouth}>
            <rect x="85" y="72" width="30" height="3" rx="1.5" fill="#00ff88" opacity="0.6" />
          </g>

          {/* Head side lights */}
          <circle cx="58" cy="60" r="3" fill="#00ff88" filter="url(#glow)" className={styles.sideLight} />
          <circle cx="142" cy="60" r="3" fill="#00ff88" filter="url(#glow)" className={styles.sideLight} />
        </g>

        {/* Neck */}
        <rect x="85" y="95" width="30" height="15" rx="3" fill="url(#bodyGradient)" stroke="#333" strokeWidth="1" />
        <rect x="90" y="98" width="20" height="2" fill="#00ff88" opacity="0.5" />
        <rect x="90" y="103" width="20" height="2" fill="#00ff88" opacity="0.5" />

        {/* Body/Torso */}
        <g className={styles.robotBody}>
          {/* Main torso */}
          <path
            d="M50 110 L60 110 L65 115 L135 115 L140 110 L150 110 L155 120 L155 190 L145 200 L55 200 L45 190 L45 120 Z"
            fill="url(#bodyGradient)"
            stroke="#00ff88"
            strokeWidth="1.5"
          />

          {/* Chest plate */}
          <rect x="70" y="125" width="60" height="50" rx="8" fill="#0a0a15" stroke="#00aaff" strokeWidth="1" opacity="0.8" />

          {/* Core energy */}
          <circle cx="100" cy="150" r="18" fill="url(#coreGlow)" filter="url(#strongGlow)" className={styles.core} />
          <circle cx="100" cy="150" r="10" fill="#00ff88" className={styles.coreInner} />
          <circle cx="100" cy="150" r="5" fill="#ffffff" opacity="0.8" />

          {/* Chest details */}
          <rect x="72" y="178" width="56" height="3" rx="1.5" fill="#00ff88" opacity="0.4" />
          <rect x="72" y="184" width="40" height="2" rx="1" fill="#00aaff" opacity="0.3" />

          {/* Shoulder joints */}
          <circle cx="45" cy="120" r="10" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />
          <circle cx="155" cy="120" r="10" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />
          <circle cx="45" cy="120" r="4" fill="#00aaff" filter="url(#glow)" />
          <circle cx="155" cy="120" r="4" fill="#00aaff" filter="url(#glow)" />
        </g>

        {/* Arms */}
        <g className={styles.leftArm}>
          {/* Upper arm */}
          <rect x="20" y="120" width="20" height="45" rx="5" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />
          <rect x="24" y="130" width="12" height="3" fill="#00ff88" opacity="0.4" />

          {/* Elbow joint */}
          <circle cx="30" cy="170" r="8" fill="url(#bodyGradient)" stroke="#00aaff" strokeWidth="1" />

          {/* Lower arm */}
          <rect x="22" y="175" width="16" height="40" rx="4" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />

          {/* Hand */}
          <ellipse cx="30" cy="220" rx="10" ry="8" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />
          <circle cx="30" cy="220" r="4" fill="#00aaff" filter="url(#glow)" />
        </g>

        <g className={styles.rightArm}>
          {/* Upper arm */}
          <rect x="160" y="120" width="20" height="45" rx="5" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />
          <rect x="164" y="130" width="12" height="3" fill="#00ff88" opacity="0.4" />

          {/* Elbow joint */}
          <circle cx="170" cy="170" r="8" fill="url(#bodyGradient)" stroke="#00aaff" strokeWidth="1" />

          {/* Lower arm */}
          <rect x="162" y="175" width="16" height="40" rx="4" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />

          {/* Hand */}
          <ellipse cx="170" cy="220" rx="10" ry="8" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />
          <circle cx="170" cy="220" r="4" fill="#00aaff" filter="url(#glow)" />
        </g>

        {/* Waist */}
        <rect x="60" y="200" width="80" height="15" rx="5" fill="url(#bodyGradient)" stroke="#333" strokeWidth="1" />
        <rect x="80" y="204" width="40" height="3" fill="#00ff88" opacity="0.3" />

        {/* Legs */}
        <g className={styles.leftLeg}>
          {/* Hip joint */}
          <circle cx="75" cy="220" r="8" fill="url(#bodyGradient)" stroke="#00aaff" strokeWidth="1" />

          {/* Upper leg */}
          <rect x="65" y="225" width="20" height="35" rx="5" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />

          {/* Knee joint */}
          <circle cx="75" cy="265" r="6" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />

          {/* Lower leg */}
          <rect x="67" y="268" width="16" height="30" rx="4" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />

          {/* Foot */}
          <ellipse cx="75" cy="302" rx="14" ry="6" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />
        </g>

        <g className={styles.rightLeg}>
          {/* Hip joint */}
          <circle cx="125" cy="220" r="8" fill="url(#bodyGradient)" stroke="#00aaff" strokeWidth="1" />

          {/* Upper leg */}
          <rect x="115" y="225" width="20" height="35" rx="5" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />

          {/* Knee joint */}
          <circle cx="125" cy="265" r="6" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />

          {/* Lower leg */}
          <rect x="117" y="268" width="16" height="30" rx="4" fill="url(#bodyGradient)" stroke="#444" strokeWidth="1" />

          {/* Foot */}
          <ellipse cx="125" cy="302" rx="14" ry="6" fill="url(#bodyGradient)" stroke="#00ff88" strokeWidth="1" />
        </g>

        {/* Ground shadow */}
        <ellipse cx="100" cy="310" rx="50" ry="8" fill="rgba(0, 255, 136, 0.1)" className={styles.shadow} />
      </svg>

      {/* Particle effects around robot */}
      <div className={styles.particles}>
        <span className={styles.particle}></span>
        <span className={styles.particle}></span>
        <span className={styles.particle}></span>
        <span className={styles.particle}></span>
        <span className={styles.particle}></span>
        <span className={styles.particle}></span>
      </div>
    </div>
  );
}

function HeroSection() {
  return (
    <section className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.gridLines}></div>
        <div className={styles.glowOrb}></div>
        <div className={styles.glowOrb2}></div>
      </div>

      <div className={styles.heroWrapper}>
        <div className={styles.heroContent}>
          <div className={styles.badge}>
            <span className={styles.badgeIcon}>◈</span>
            <span>Vectra - Physical AI & Robotics</span>
          </div>
          <h1 className={styles.heroTitle}>
            <span className={styles.titleLine}>AI learned to think.</span>
            <span className={styles.titleLine}>
              <span className={styles.highlight}>Now teach it to move.</span>
            </span>
          </h1>
          <p className={styles.heroSubtitle}>
            Master the complete pipeline from ROS 2 foundations to voice-controlled
            humanoid robots. Build autonomous systems that perceive, reason, and act
            in the physical world.
          </p>
          <div className={styles.stats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>10</span>
              <span className={styles.statLabel}>Chapters</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>80+</span>
              <span className={styles.statLabel}>Lessons</span>
            </div>
            <div className={styles.statDivider}></div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>4</span>
              <span className={styles.statLabel}>Parts</span>
            </div>
          </div>
          <div className={styles.ctaGroup}>
            <Link className={styles.ctaPrimary} to="/docs/intro">
              <span>Start Learning</span>
              <svg className={styles.ctaArrow} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M5 12h14M12 5l7 7-7 7"/>
              </svg>
            </Link>
            <Link className={styles.ctaSecondary} to="/docs/Part-1-ROS2-Foundation">
              View Curriculum
            </Link>
          </div>
        </div>

        <div className={styles.heroRobot}>
          <AnimatedRobot />
        </div>
      </div>

      <div className={styles.scrollIndicator}>
        <span>Scroll to explore</span>
        <div className={styles.scrollLine}></div>
      </div>
    </section>
  );
}

function TechStackSection() {
  const technologies = [
    { name: 'ROS 2', icon: '🤖' },
    { name: 'Python', icon: '🐍' },
    { name: 'Gazebo', icon: '🌍' },
    { name: 'Unity', icon: '🎮' },
    { name: 'Isaac Sim', icon: '⚡' },
    { name: 'Nav2', icon: '🧭' },
    { name: 'VSLAM', icon: '👁️' },
    { name: 'LLMs', icon: '🧠' },
  ];

  return (
    <section className={styles.techStack}>
      <div className={styles.techStackContent}>
        <p className={styles.techLabel}>Technologies You'll Master</p>
        <div className={styles.techScroll}>
          <div className={styles.techTrack}>
            {[...technologies, ...technologies].map((tech, index) => (
              <div key={index} className={styles.techItem}>
                <span className={styles.techIcon}>{tech.icon}</span>
                <span className={styles.techName}>{tech.name}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

function JourneySection() {
  const parts = [
    {
      number: '01',
      title: 'The Robotic Nervous System',
      subtitle: 'ROS 2 Foundation',
      description: 'Master nodes, topics, services, and actions. Build the communication backbone that connects every sensor and actuator.',
      chapters: ['ROS 2 Core Concepts', 'Python Development with rclpy', 'URDF & Humanoid Modeling'],
      color: '#00ff88',
      link: '/docs/Part-1-ROS2-Foundation',
    },
    {
      number: '02',
      title: 'The Digital Twin',
      subtitle: 'Simulation Mastery',
      description: 'Create high-fidelity virtual environments. Test algorithms safely before deploying to real hardware.',
      chapters: ['Gazebo Physics Simulation', 'Unity for HRI', 'Sensor Simulation'],
      color: '#00d4ff',
      link: '/docs/Part-2-Digital-Twin',
    },
    {
      number: '03',
      title: 'The AI Brain',
      subtitle: 'Advanced Perception',
      description: 'Harness GPU-accelerated AI for perception. Enable your robot to see, map, and navigate autonomously.',
      chapters: ['NVIDIA Isaac Sim', 'Visual SLAM', 'Nav2 Navigation'],
      color: '#ff6b6b',
      link: '/docs/Part-3-Advanced-Simulation-Perception',
    },
    {
      number: '04',
      title: 'Voice to Action',
      subtitle: 'VLA Pipeline',
      description: 'Connect language models to robot control. Speak commands, watch robots execute complex tasks.',
      chapters: ['Speech Recognition', 'LLM Task Planning', 'Action Execution'],
      color: '#ffd93d',
      link: '/docs/Part-4-Vision-Language-Action',
    },
  ];

  return (
    <section className={styles.journey}>
      <div className={styles.journeyContent}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>The Learning Path</span>
          <h2 className={styles.sectionTitle}>Your Journey to Physical AI</h2>
          <p className={styles.sectionSubtitle}>
            Four comprehensive parts take you from fundamentals to building
            voice-controlled autonomous humanoids.
          </p>
        </div>
        <div className={styles.partsGrid}>
          {parts.map((part, index) => (
            <Link
              key={part.number}
              to={part.link}
              className={styles.partCard}
              style={{ '--card-accent': part.color } as React.CSSProperties}
            >
              <div className={styles.partHeader}>
                <span className={styles.partNumber}>{part.number}</span>
                <span className={styles.partSubtitle}>{part.subtitle}</span>
              </div>
              <h3 className={styles.partTitle}>{part.title}</h3>
              <p className={styles.partDescription}>{part.description}</p>
              <div className={styles.partChapters}>
                {part.chapters.map((chapter, i) => (
                  <span key={i} className={styles.chapterTag}>{chapter}</span>
                ))}
              </div>
              <div className={styles.partFooter}>
                <span className={styles.exploreLink}>
                  Explore Part {part.number}
                  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M5 12h14M12 5l7 7-7 7"/>
                  </svg>
                </span>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🎯',
      title: 'Project-Based Learning',
      description: 'Every chapter builds toward a working capstone project. Learn by doing, not just reading.',
    },
    {
      icon: '🔧',
      title: 'Production-Ready Code',
      description: 'All examples use industry-standard tools and practices. Code that actually runs on real robots.',
    },
    {
      icon: '📊',
      title: 'Progressive Complexity',
      description: 'Start simple, build complexity. Each lesson builds on the last with clear prerequisites.',
    },
    {
      icon: '🌐',
      title: 'End-to-End Pipeline',
      description: 'From sensor data to motor commands. Understand the complete robotics software stack.',
    },
    {
      icon: '🤝',
      title: 'Human-Robot Interaction',
      description: 'Build robots that communicate naturally through voice, gestures, and intuitive interfaces.',
    },
    {
      icon: '⚡',
      title: 'GPU Acceleration',
      description: 'Leverage NVIDIA Isaac for real-time perception. Train and deploy AI at robot speed.',
    },
  ];

  return (
    <section className={styles.features}>
      <div className={styles.featuresContent}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>Why This Book</span>
          <h2 className={styles.sectionTitle}>Built for Real-World Robotics</h2>
          <p className={styles.sectionSubtitle}>
            Not another theory-heavy textbook. This is a hands-on guide to building
            robots that work.
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <span className={styles.featureIcon}>{feature.icon}</span>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function DemoSection() {
  return (
    <section className={styles.demo}>
      <div className={styles.demoContent}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>The End Goal</span>
          <h2 className={styles.sectionTitle}>Voice to Action Pipeline</h2>
          <p className={styles.sectionSubtitle}>
            By the end of this book, you'll build a complete system that turns
            natural language into robot actions.
          </p>
        </div>
        <div className={styles.pipelineDemo}>
          <div className={styles.pipelineStep}>
            <div className={styles.stepIcon}>🎤</div>
            <div className={styles.stepContent}>
              <span className={styles.stepLabel}>Input</span>
              <p className={styles.stepText}>"Pick up the red cup and place it on the table"</p>
            </div>
          </div>
          <div className={styles.pipelineArrow}>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </div>
          <div className={styles.pipelineStep}>
            <div className={styles.stepIcon}>🧠</div>
            <div className={styles.stepContent}>
              <span className={styles.stepLabel}>LLM Planning</span>
              <p className={styles.stepText}>Task decomposition → Action sequence → Motion planning</p>
            </div>
          </div>
          <div className={styles.pipelineArrow}>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </div>
          <div className={styles.pipelineStep}>
            <div className={styles.stepIcon}>🤖</div>
            <div className={styles.stepContent}>
              <span className={styles.stepLabel}>Execution</span>
              <p className={styles.stepText}>Navigate → Grasp → Transport → Place</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function PrerequisitesSection() {
  return (
    <section className={styles.prerequisites}>
      <div className={styles.prerequisitesContent}>
        <div className={styles.prereqCard}>
          <h3 className={styles.prereqTitle}>What You Need</h3>
          <ul className={styles.prereqList}>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Basic Python programming experience</span>
            </li>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Familiarity with Linux command line</span>
            </li>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Understanding of basic ML concepts (helpful but not required)</span>
            </li>
            <li>
              <span className={styles.checkIcon}>✓</span>
              <span>Curiosity and willingness to experiment</span>
            </li>
          </ul>
        </div>
        <div className={styles.prereqCard}>
          <h3 className={styles.prereqTitle}>What You'll Gain</h3>
          <ul className={styles.prereqList}>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Deep understanding of robotics middleware (ROS 2)</span>
            </li>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Ability to build and simulate humanoid robots</span>
            </li>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Skills in AI-powered perception and navigation</span>
            </li>
            <li>
              <span className={styles.starIcon}>★</span>
              <span>Complete voice-to-action system implementation</span>
            </li>
          </ul>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className={styles.ctaContent}>
        <div className={styles.ctaGlow}></div>
        <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
        <p className={styles.ctaText}>
          Start your journey from digital AI to physical robotics.
          No prior robotics experience required.
        </p>
        <div className={styles.ctaButtons}>
          <Link className={styles.ctaButtonPrimary} to="/docs/intro">
            <span>Begin Your Journey</span>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M5 12h14M12 5l7 7-7 7"/>
            </svg>
          </Link>
          <Link className={styles.ctaButtonSecondary} to="/docs/Part-1-ROS2-Foundation/ros2-nodes-topics-services">
            Jump to Chapter 1
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout
      title="From Digital AI to Physical Robotics"
      description="Vectra - Master the complete pipeline from ROS 2 foundations to voice-controlled humanoid robots. Build autonomous systems that perceive, reason, and act in the physical world.">
      <main className={styles.main}>
        <HeroSection />
        <TechStackSection />
        <JourneySection />
        <FeaturesSection />
        <DemoSection />
        <PrerequisitesSection />
        <CTASection />
      </main>
    </Layout>
  );
}
