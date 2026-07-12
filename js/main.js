// ============================================================
// Shared interactions: theme, nav, reveals, counters, typer,
// tilt cards, timeline progress, project filter.
// Used by both index.html and teaching.html (missing elements
// are simply skipped).
// ============================================================
(function () {
  "use strict";
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const $ = (sel, root) => (root || document).querySelector(sel);
  const $$ = (sel, root) => Array.from((root || document).querySelectorAll(sel));

  /* ---------- theme toggle (persisted) ---------- */
  const root = document.documentElement;
  const savedTheme = localStorage.getItem("pt-theme");
  if (savedTheme === "light" || savedTheme === "dark") root.dataset.theme = savedTheme;

  const themeToggle = $("#themeToggle");
  if (themeToggle) {
    themeToggle.addEventListener("click", () => {
      root.dataset.theme = root.dataset.theme === "dark" ? "light" : "dark";
      localStorage.setItem("pt-theme", root.dataset.theme);
    });
  }

  /* ---------- mobile menu ---------- */
  const burger = $("#navBurger");
  const navLinks = $("#navLinks");
  if (burger && navLinks) {
    burger.addEventListener("click", () => {
      const open = navLinks.classList.toggle("is-open");
      document.body.classList.toggle("menu-open", open);
      burger.setAttribute("aria-expanded", open);
    });
    navLinks.addEventListener("click", (e) => {
      if (e.target.tagName === "A") {
        navLinks.classList.remove("is-open");
        document.body.classList.remove("menu-open");
        burger.setAttribute("aria-expanded", "false");
      }
    });
  }

  /* ---------- scroll progress bar ---------- */
  const scrollBar = $("#scrollBar");
  if (scrollBar) {
    const onScroll = () => {
      const max = document.documentElement.scrollHeight - window.innerHeight;
      scrollBar.style.width = (max > 0 ? (window.scrollY / max) * 100 : 0) + "%";
    };
    window.addEventListener("scroll", onScroll, { passive: true });
    onScroll();
  }

  /* ---------- reveal on scroll ---------- */
  const revealObserver = new IntersectionObserver(
    (entries) => {
      entries.forEach((entry) => {
        if (entry.isIntersecting) {
          entry.target.classList.add("is-visible");
          revealObserver.unobserve(entry.target);
        }
      });
    },
    { threshold: 0.12, rootMargin: "0px 0px -40px 0px" }
  );
  $$(".reveal").forEach((el) => revealObserver.observe(el));

  /* ---------- hero name letter drop ---------- */
  const heroName = $("#heroName");
  if (heroName && !reducedMotion) {
    const words = heroName.textContent.trim().split(/[\s\u00A0]+/);
    heroName.textContent = "";
    let ci = 0;
    words.forEach((word, wi) => {
      const w = document.createElement("span");
      w.className = "word";
      [...word].forEach((ch) => {
        const span = document.createElement("span");
        span.className = "ch";
        span.style.animationDelay = 0.06 * ci++ + "s";
        span.textContent = ch;
        w.appendChild(span);
      });
      heroName.appendChild(w);
      if (wi < words.length - 1) heroName.appendChild(document.createTextNode(" "));
    });
  }

  /* ---------- role typer ---------- */
  const typer = $("#roleTyper");
  if (typer && !reducedMotion) {
    const roles = ["researcher", "civil engineer", "IoT maker", "concrete whisperer", "teacher", "prototype builder"];
    let roleIdx = 0, charIdx = roles[0].length, deleting = false;
    (function tick() {
      const word = roles[roleIdx];
      typer.textContent = word.slice(0, charIdx);
      let delay = deleting ? 45 : 90;
      if (!deleting && charIdx === word.length) { delay = 2200; deleting = true; }
      else if (deleting && charIdx === 0) { deleting = false; roleIdx = (roleIdx + 1) % roles.length; delay = 350; }
      else charIdx += deleting ? -1 : 1;
      setTimeout(tick, delay);
    })();
  }

  /* ---------- animated counters ---------- */
  const counterObserver = new IntersectionObserver(
    (entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        const el = entry.target;
        counterObserver.unobserve(el);
        const target = parseInt(el.dataset.count, 10);
        const suffix = el.dataset.suffix || "";
        if (reducedMotion) { el.innerHTML = target + suffix; return; }
        const t0 = performance.now();
        (function step(now) {
          const p = Math.min((now - t0) / 1400, 1);
          const eased = 1 - Math.pow(1 - p, 3);
          el.innerHTML = Math.round(target * eased) + suffix;
          if (p < 1) requestAnimationFrame(step);
        })(t0);
      });
    },
    { threshold: 0.6 }
  );
  $$("[data-count]").forEach((el) => counterObserver.observe(el));

  /* ---------- 3D tilt on cards ---------- */
  if (!reducedMotion && matchMedia("(pointer: fine)").matches) {
    $$("[data-tilt]").forEach((card) => {
      card.addEventListener("pointermove", (e) => {
        const r = card.getBoundingClientRect();
        const px = (e.clientX - r.left) / r.width - 0.5;
        const py = (e.clientY - r.top) / r.height - 0.5;
        card.style.transform = `perspective(700px) rotateY(${px * 7}deg) rotateX(${-py * 7}deg) translateY(-3px)`;
      });
      card.addEventListener("pointerleave", () => { card.style.transform = ""; });
    });
  }

  /* ---------- active nav link + timeline progress ---------- */
  const sections = $$("main section[id]");
  const navAnchors = $$('.nav__links a[data-section]');
  const timelineProgress = $("#timelineProgress");
  const timeline = $(".timeline");

  window.addEventListener(
    "scroll",
    () => {
      // nav highlight
      let current = "";
      for (const sec of sections) {
        if (sec.getBoundingClientRect().top < window.innerHeight * 0.4) current = sec.id;
      }
      navAnchors.forEach((a) => a.classList.toggle("is-active", a.dataset.section === current));

      // timeline fill follows scroll
      if (timeline && timelineProgress) {
        const r = timeline.getBoundingClientRect();
        const visible = Math.min(Math.max(window.innerHeight * 0.75 - r.top, 0), r.height);
        timelineProgress.style.height = (visible / r.height) * 100 + "%";
      }
    },
    { passive: true }
  );

  /* ---------- project filter ---------- */
  const chips = $$(".filter-bar .chip");
  const cards = $$(".project-card");
  chips.forEach((chip) => {
    chip.addEventListener("click", () => {
      chips.forEach((c) => c.classList.remove("is-active"));
      chip.classList.add("is-active");
      const f = chip.dataset.filter;
      cards.forEach((card) => {
        const show = f === "all" || card.dataset.cat.split(" ").includes(f);
        card.classList.toggle("is-hidden", !show);
      });
    });
  });

  /* ---------- footer year ---------- */
  const year = $("#year");
  if (year) year.textContent = new Date().getFullYear();
})();
