# paveltravnicek.com — personal website & student zone

A fast, dependency-free personal website for **Pavel Trávníček** — interactive CV,
project portfolio and a **Student Zone** with course requirements, homework and
live deadline countdowns.

**Theme: “Concrete × Circuits”** — a blueprint-inspired design reflecting work at
the intersection of civil engineering and IoT. Dark (night blueprint) and light
(paper blueprint) modes, with an interactive Three.js hero scene: a wave field of
concrete columns with pulsing sensor nodes. Click the grid to drop blocks into it. 🧱

## Structure

```
index.html        Main page: hero, about, skills, timeline, projects, contact
teaching.html     Student Zone: courses, requirements, homework, materials
css/style.css     All styling + design tokens (dark/light themes)
js/hero3d.js      Three.js hero scene
js/vendor/        Vendored three.js r160 (no CDN dependency)
js/main.js        Shared interactions (theme, nav, reveals, counters, tilt…)
js/teaching.js    Renders the Student Zone from the data file
data/courses.js   ✏️ THE file to edit for course/homework updates
```

No build step, no npm, no framework. Edit a file, refresh the browser, done.

## Updating content

### Courses, homework, deadlines (most frequent)
Everything students see comes from **`data/courses.js`**. It is heavily commented.
Change a `due:` date and the countdown badge updates itself; set `done: true`
to close a homework. **The current course content is sample/placeholder data —
replace codes, rooms, dates and links before going live.**

### CV content
Edit the HTML sections in `index.html` directly (about text, timeline entries,
project cards, contact details). To add your photo, drop it in `assets/` and
replace the placeholder SVG inside `.portrait-card__frame`.

### Colors / fonts
All design tokens are CSS variables at the top of `css/style.css`.

## Running locally

Any static server works:

```bash
python3 -m http.server 8000
# → http://localhost:8000
```

(Opening `index.html` via `file://` mostly works too, but the 3D scene needs a
server because it uses ES modules.)

## Deploying

The site is 100% static — host it anywhere:

- **GitHub Pages**: Settings → Pages → deploy from `main` branch, root folder.
  Note: Pages on a *private* repo requires GitHub Pro; otherwise make the repo
  public or use one of the options below.
- **Cloudflare Pages / Netlify** (both free for private repos): connect the repo,
  no build command, output directory = repo root.
- **Custom domain**: point `www.paveltravnicek.com` at your host via a CNAME
  record; on GitHub Pages also add the domain in Settings → Pages.

## Credits

Fonts: [Space Grotesk](https://fonts.google.com/specimen/Space+Grotesk) &
[JetBrains Mono](https://fonts.google.com/specimen/JetBrains+Mono) (Google Fonts).
3D: [three.js](https://threejs.org/) r160 (vendored in `js/vendor/`). Everything else: hand-written HTML/CSS/JS.
