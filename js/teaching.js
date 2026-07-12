// ============================================================
// Student Zone renderer — reads TEACHING_DATA (data/courses.js)
// and builds the course cards, tabs, deadline badges and the
// "next deadline" spotlight. No framework, no build step.
// ============================================================
(function () {
  "use strict";
  if (typeof TEACHING_DATA === "undefined") return;

  const root = document.getElementById("coursesRoot");
  const MS_DAY = 24 * 60 * 60 * 1000;
  const today = new Date();
  today.setHours(0, 0, 0, 0);

  const esc = (s) =>
    String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

  const fmtDate = (iso) =>
    new Date(iso + "T00:00:00").toLocaleDateString("en-GB", { day: "numeric", month: "short", year: "numeric" });

  function daysUntil(iso) {
    return Math.round((new Date(iso + "T00:00:00") - today) / MS_DAY);
  }

  function badge(hw) {
    if (hw.done) return '<span class="badge badge--done">✓ closed</span>';
    const d = daysUntil(hw.due);
    if (d < 0) return '<span class="badge badge--over">overdue</span>';
    if (d === 0) return '<span class="badge badge--soon">due TODAY</span>';
    if (d <= 7) return `<span class="badge badge--soon">due in ${d} day${d === 1 ? "" : "s"}</span>`;
    return `<span class="badge badge--ok">in ${d} days</span>`;
  }

  /* ---------- announcement ---------- */
  const ann = TEACHING_DATA.announcement;
  if (ann && ann.title) {
    document.getElementById("announcementTitle").textContent = ann.title;
    document.getElementById("announcementText").textContent = ann.text || "";
    document.getElementById("announcement").hidden = false;
  }

  /* ---------- office hours ---------- */
  const oh = TEACHING_DATA.office_hours;
  if (oh) {
    document.getElementById("officeHours").textContent = `${oh.when} — ${oh.where}`;
  }

  /* ---------- next deadline spotlight ---------- */
  const upcoming = TEACHING_DATA.courses
    .flatMap((c) => c.homework.filter((h) => !h.done).map((h) => ({ ...h, course: c.title })))
    .filter((h) => daysUntil(h.due) >= 0)
    .sort((a, b) => a.due.localeCompare(b.due));

  if (upcoming.length) {
    const next = upcoming[0];
    const d = daysUntil(next.due);
    document.getElementById("deadlineSpotlightInner").innerHTML =
      `<span aria-hidden="true">⏰</span> Next deadline: <b>${esc(next.title)}</b>` +
      ` <span class="mono">(${esc(next.course)})</span> — ${fmtDate(next.due)} ${badge(next)}`;
    document.getElementById("deadlineSpotlight").hidden = false;
  }

  /* ---------- course cards ---------- */
  TEACHING_DATA.courses.forEach((c, idx) => {
    const el = document.createElement("article");
    el.className = "course reveal";
    el.id = c.id;

    const hwCount = c.homework.filter((h) => !h.done && daysUntil(h.due) >= 0).length;

    el.innerHTML = `
      <button class="course__head" aria-expanded="false" aria-controls="${esc(c.id)}-body">
        <span class="course__icon" aria-hidden="true">${esc(c.icon || "📘")}</span>
        <span class="course__title-wrap">
          <span class="course__code mono">${esc(c.code)}</span>
          <h2>${esc(c.title)}</h2>
          <span class="course__meta">
            <span>${esc(c.semester)}</span>
            <span>${esc(c.role)}</span>
            <span>${esc(c.schedule)}</span>
            ${hwCount ? `<span><b>${hwCount}</b> open deadline${hwCount === 1 ? "" : "s"}</span>` : ""}
          </span>
        </span>
        <svg class="course__chevron" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" aria-hidden="true"><path d="m6 9 6 6 6-6"/></svg>
      </button>
      <div class="course__body" id="${esc(c.id)}-body">
        <div class="course__body-inner"><div class="course__body-content">
          <div class="course__tabs" role="tablist">
            <button class="course__tab is-active" data-pane="overview">overview</button>
            <button class="course__tab" data-pane="requirements">requirements</button>
            <button class="course__tab" data-pane="homework">homework${hwCount ? ` (${hwCount})` : ""}</button>
            <button class="course__tab" data-pane="materials">materials</button>
          </div>

          <div class="course__pane is-active" data-pane="overview">
            ${c.overview.map((p) => `<p>${esc(p)}</p>`).join("")}
          </div>

          <div class="course__pane" data-pane="requirements">
            <ul class="req-list">
              ${c.requirements.map((r) => `<li><span><b>${esc(r.label)}:</b> ${esc(r.detail)}</span></li>`).join("")}
            </ul>
            ${
              c.grading && c.grading.length
                ? `<div class="grading" role="img" aria-label="Grading breakdown: ${esc(
                    c.grading.map((g) => `${g.label} ${g.pct}%`).join(", ")
                  )}">` +
                  c.grading
                    .map((g) => `<div class="grading__seg" style="width:${g.pct}%" title="${esc(g.label)} — ${g.pct}%">${g.pct >= 15 ? esc(g.label) + " " + g.pct + "%" : g.pct + "%"}</div>`)
                    .join("") +
                  `</div>`
                : ""
            }
          </div>

          <div class="course__pane" data-pane="homework">
            ${
              c.homework.length
                ? `<ul class="hw-list">` +
                  c.homework
                    .map(
                      (h) => `
                    <li class="hw-item">
                      <div class="hw-item__main">
                        <div class="hw-item__title">${esc(h.title)}</div>
                        <div class="hw-item__desc">${esc(h.desc)}</div>
                      </div>
                      <span class="hw-item__due">${fmtDate(h.due)}${h.points ? ` · ${h.points} pts` : ""}</span>
                      ${badge(h)}
                      ${h.link && h.link !== "#" ? `<a class="hw-item__link" href="${esc(h.link)}" target="_blank" rel="noopener">open →</a>` : ""}
                    </li>`
                    )
                    .join("") +
                  `</ul>`
                : `<p>No homework here — this one is all about the long game. 🎯</p>`
            }
          </div>

          <div class="course__pane" data-pane="materials">
            <ul class="mat-list">
              ${c.materials
                .map(
                  (m) =>
                    `<li><a href="${esc(m.url)}" ${m.url.startsWith("http") ? 'target="_blank" rel="noopener"' : ""}><span class="mat-type">${esc(m.type)}</span> ${esc(m.label)}</a></li>`
                )
                .join("")}
            </ul>
            <p style="margin-top:1rem;font-size:0.8rem;opacity:0.7">Links marked <span class="mat-type mono">#</span> are placeholders — real links coming with the semester.</p>
          </div>
        </div></div>
      </div>`;

    root.appendChild(el);

    /* accordion */
    const head = el.querySelector(".course__head");
    head.addEventListener("click", () => {
      const open = el.classList.toggle("is-open");
      head.setAttribute("aria-expanded", open);
    });

    /* tabs */
    el.querySelectorAll(".course__tab").forEach((tab) => {
      tab.addEventListener("click", () => {
        el.querySelectorAll(".course__tab").forEach((t) => t.classList.remove("is-active"));
        el.querySelectorAll(".course__pane").forEach((p) => p.classList.remove("is-active"));
        tab.classList.add("is-active");
        el.querySelector(`.course__pane[data-pane="${tab.dataset.pane}"]`).classList.add("is-active");
      });
    });

    /* first course opens by default */
    if (idx === 0) {
      el.classList.add("is-open");
      head.setAttribute("aria-expanded", "true");
    }
  });

  /* deep link: teaching.html#course-id opens that course */
  if (location.hash) {
    const target = document.getElementById(location.hash.slice(1));
    if (target && target.classList.contains("course")) {
      target.classList.add("is-open");
      target.querySelector(".course__head").setAttribute("aria-expanded", "true");
      setTimeout(() => target.scrollIntoView({ behavior: "smooth", block: "start" }), 150);
    }
  }
})();
