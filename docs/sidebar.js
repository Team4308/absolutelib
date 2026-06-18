

const SIDEBAR_VERSION = "2.1.4"; // Used to verify deployment success

// Sidebar HTML as a string
// Dynamically resolve relative paths for sidebar links
function getSidebarLinks() {
    // Robust root detection: use the location of this script as an anchor
    const sidebarScript = Array.from(document.scripts).find(s => s.src.includes('docs/sidebar.js'));
    const scriptUrl = new URL(sidebarScript.src);
    
    // Explicit production vs local root detection
    let siteRoot = scriptUrl.pathname.replace(/docs\/sidebar\.js$/, '');
    
    // Safety for GitHub Pages - ensure we include the repo name if needed
    if (window.location.hostname.includes('github.io') && !siteRoot.startsWith('/absolutelib/')) {
        siteRoot = '/absolutelib/';
    }
    
    // Use the siteRoot as a reliable absolute prefix for all links
    const link = (href) => siteRoot + href;

    console.log(`[Sidebar ${SIDEBAR_VERSION}] Detected siteRoot: ${siteRoot}`);
    
    const path = window.location.pathname;
    
    // Helper to determine if a link is active
    const isActive = (href) => {
        const fullHref = link(href);
        const cleanPath = path.replace(/\/index\.html$/, '/').replace(/\/$/, '');
        const cleanHref = fullHref.replace(/\/index\.html$/, '/').replace(/\/$/, '');
        return cleanPath === cleanHref;
    };

    return `
    <div class="sidebar" id="sidebar">
        <div class="brand">
            <h1>AbsoluteLib V2</h1>
            <span class="version">v...</span>
        </div>
        <div class="nav-links">
            <div class="nav-group">
                <div class="nav-group-title">Getting Started</div>
                <a href="${link('index.html')}" class="nav-item ${isActive('index.html') ? 'active' : ''}">Introduction</a>
                <a href="${link('docs/installation.html')}" class="nav-item ${isActive('docs/installation.html') ? 'active' : ''}">Installation</a>
            </div>
            <div class="nav-group">
                <div class="nav-group-title">Core</div>
                <a href="${link('docs/wrappers.html')}" class="nav-item ${isActive('docs/wrappers.html') ? 'active' : ''}">Hardware Wrappers</a>
                <a href="${link('docs/subsystems.html')}" class="nav-item ${isActive('docs/subsystems.html') ? 'active' : ''}">Subsystems</a>
                <a href="${link('docs/vision.html')}" class="nav-item ${isActive('docs/vision.html') ? 'active' : ''}">Vision</a>
            </div>
            <div class="nav-group">
                <div class="nav-group-title">Utilities</div>
                <a href="${link('docs/utilities/leds.html')}" class="nav-item ${isActive('docs/utilities/leds.html') ? 'active' : ''}">LEDs</a>
                <a href="${link('docs/utilities/math.html')}" class="nav-item ${isActive('docs/utilities/math.html') ? 'active' : ''}">Math</a>
                <a href="${link('docs/utilities/input.html')}" class="nav-item ${isActive('docs/utilities/input.html') ? 'active' : ''}">Input</a>
                <a href="${link('docs/utilities/encoders.html')}" class="nav-item ${isActive('docs/utilities/encoders.html') ? 'active' : ''}">Encoders</a>
                <a href="${link('docs/utilities/tuning.html')}" class="nav-item ${isActive('docs/utilities/tuning.html') ? 'active' : ''}">Tuning</a>
                <a href="${link('docs/utilities/pathing.html')}" class="nav-item ${isActive('docs/utilities/pathing.html') ? 'active' : ''}">Pathing</a>
                <a href="${link('docs/utilities/music.html')}" class="nav-item ${isActive('docs/utilities/music.html') ? 'active' : ''}">Music</a>
                <a href="${link('docs/utilities/commands.html')}" class="nav-item ${isActive('docs/utilities/commands.html') ? 'active' : ''}">Commands</a>
                <a href="${link('docs/utilities/crt-solver.html')}" class="nav-item ${isActive('docs/utilities/crt-solver.html') ? 'active' : ''}">CRT Solver</a>
            </div>
            <div class="nav-group">
                <div class="nav-group-title">Trajectory</div>
                <a href="${link('docs/trajectory.html')}" class="nav-item ${isActive('docs/trajectory.html') ? 'active' : ''}">Trajectory Solver</a>
                <a href="${link('docs/shooter.html')}" class="nav-item ${isActive('docs/shooter.html') ? 'active' : ''}">Shooter System</a>
                <a href="${link('docs/obstacles.html')}" class="nav-item ${isActive('docs/obstacles.html') ? 'active' : ''}">Obstacle Avoidance</a>
            </div>
            <div class="nav-group">
                <div class="nav-group-title">Tools</div>
                <a href="${link('trajectory-debug.html')}" class="nav-item ${isActive('trajectory-debug.html') ? 'active' : ''}">Trajectory Debug</a>
                <a href="${link('changelog-viewer.html')}" class="nav-item ${isActive('changelog-viewer.html') ? 'active' : ''}">Changelog</a>
            </div>
        </div>
        <div class="sidebar-footer">
            <a href="${link('docs/javadoc/index.html')}" class="javadoc-link" target="_blank">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                    <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
                    <polyline points="14 2 14 8 20 8"></polyline>
                </svg>
                API Javadoc
            </a>
        </div>
    </div>
    `;
}
const version_url = 'https://raw.githubusercontent.com/Team4308/absolutelib/refs/heads/master/gradle.properties';

function injectSidebar() {
    let container = document.getElementById('sidebar-container');
    if (!container) {
        container = document.createElement('div');
        container.id = 'sidebar-container';
        document.body.insertBefore(container, document.body.firstChild);
    }
    container.innerHTML = getSidebarLinks();
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', injectSidebar);
    injectSidebar();
} else {
    injectSidebar();
}

fetchVersion();
async function fetchVersion() {
    try {
        const response = await fetch(version_url);
        const text = await response.text();
        const versionMatch = text.match(/version\s*=\s*(.+)/);
        if (versionMatch) {
            const version = versionMatch[1].trim();
            document.querySelector('.version').textContent = 'v' + version;
        }
    } catch (error) {
        // Fail silently
    }
}