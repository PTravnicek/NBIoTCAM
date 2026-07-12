// ============================================================
// STUDENT ZONE DATA — this is the only file you need to edit
// to update courses, requirements, homework and materials.
//
// ⚠️ Everything below is SAMPLE/PLACEHOLDER content. Replace the
// course codes, names, rooms, dates and links with your real ones.
//
// Notes:
//  - `due` dates use the ISO format YYYY-MM-DD. The page shows a
//    live countdown badge automatically (upcoming / due soon /
//    closed). Set `done: true` to grey a homework out as finished.
//  - `grading` percentages should add up to 100.
//  - `materials` links can point anywhere (Moodle, PDF, GitHub…).
//  - Add or remove whole course objects freely — the page adapts.
// ============================================================

const TEACHING_DATA = {
  // Set to null to hide the announcement banner.
  announcement: {
    title: "Welcome to the new Student Zone!",
    text: "All course requirements, homework and deadlines now live here. Bookmark this page — the countdown badges never lie.",
  },

  office_hours: {
    when: "Wednesdays 14:00–15:30 (or by email appointment)",
    where: "CTU in Prague, Faculty of Civil Engineering — office B-000",
  },

  courses: [
    {
      id: "concrete-tech",
      code: "133XXX1 · SAMPLE",
      title: "Concrete Technology",
      icon: "🏗️",
      semester: "Winter 2026/27",
      role: "Lectures + labs",
      schedule: "Wed 10:00–11:40 · Room B-000",
      overview: [
        "What actually happens between mixing water hitting cement and a bridge carrying trucks? This course walks through fresh and hardened concrete behaviour, mix design, hydration, and durability — with real lab experiments, not just slides.",
        "You will design your own mix, cast it, break it, and explain the numbers. Expect to get your hands dirty (literally — bring clothes you don't love).",
      ],
      requirements: [
        { label: "Attendance", detail: "labs are mandatory (max 2 excused absences), lectures strongly recommended" },
        { label: "Lab reports", detail: "4 reports, submitted within 2 weeks of each lab session" },
        { label: "Semester project", detail: "mix design + verification of your own concrete recipe" },
        { label: "Final exam", detail: "written test + short oral discussion of your project" },
      ],
      grading: [
        { label: "Lab reports", pct: 30 },
        { label: "Project", pct: 30 },
        { label: "Final exam", pct: 40 },
      ],
      homework: [
        {
          title: "HW1 — Cement paste consistency",
          desc: "Report from the first lab: normal consistency & setting times (Vicat).",
          due: "2026-10-21",
          points: 10,
          link: "#",
        },
        {
          title: "HW2 — Aggregate grading curve",
          desc: "Sieve analysis of your assigned aggregate + grading curve plot and discussion.",
          due: "2026-11-11",
          points: 10,
          link: "#",
        },
        {
          title: "HW3 — Mix design proposal",
          desc: "First iteration of your semester-project mix design, with justification.",
          due: "2026-12-02",
          points: 15,
          link: "#",
        },
      ],
      materials: [
        { label: "Syllabus & rules (PDF)", type: "pdf", url: "#" },
        { label: "Lecture slides", type: "moodle", url: "#" },
        { label: "Lab report template", type: "docx", url: "#" },
        { label: "Recommended reading list", type: "link", url: "#" },
      ],
    },

    {
      id: "iot-civil",
      code: "133XXX2 · SAMPLE",
      title: "IoT & Monitoring for Civil Engineers",
      icon: "📡",
      semester: "Winter 2026/27",
      role: "Lectures + project studio",
      schedule: "Thu 14:00–15:40 · Lab C-000",
      overview: [
        "Structures can talk — if you give them sensors. This course covers the whole chain: sensors, microcontrollers (ESP32), low-power design, NB-IoT/LoRa connectivity, and turning raw data into engineering decisions.",
        "The centrepiece is a team project: build a working monitoring device and deploy it on a real specimen by the end of the semester. Soldering experience is welcome but not required — enthusiasm is.",
      ],
      requirements: [
        { label: "Team project", detail: "working prototype + short documentation, teams of 2–3" },
        { label: "Milestones", detail: "3 checkpoint demos during the semester (concept, prototype, deployment)" },
        { label: "Attendance", detail: "project studio sessions are mandatory" },
        { label: "Final demo day", detail: "present your device live — if it blinks, you're halfway there" },
      ],
      grading: [
        { label: "Milestones", pct: 30 },
        { label: "Prototype", pct: 40 },
        { label: "Final demo", pct: 20 },
        { label: "Documentation", pct: 10 },
      ],
      homework: [
        {
          title: "HW1 — Blink & measure",
          desc: "Get your dev kit running: LED blink + first temperature reading pushed to the class server.",
          due: "2026-10-15",
          points: 5,
          link: "#",
        },
        {
          title: "HW2 — Power budget",
          desc: "Estimate the battery life of your project device; identify the two hungriest components.",
          due: "2026-11-05",
          points: 10,
          link: "#",
        },
        {
          title: "Milestone 2 — Working prototype demo",
          desc: "Your device measures and transmits real data. Duct tape allowed, excuses not.",
          due: "2026-11-26",
          points: 20,
          link: "#",
        },
      ],
      materials: [
        { label: "Course intro & rules (PDF)", type: "pdf", url: "#" },
        { label: "ESP32 starter repository", type: "github", url: "https://github.com/PTravnicek" },
        { label: "Datasheet pack", type: "zip", url: "#" },
        { label: "Project topics list", type: "moodle", url: "#" },
      ],
    },

    {
      id: "bachelor-thesis",
      code: "133BAP · SAMPLE",
      title: "Bachelor / Master Thesis Supervision",
      icon: "🎓",
      semester: "All year",
      role: "Supervisor",
      schedule: "Individual consultations",
      overview: [
        "I supervise theses at the intersection of concrete technology and instrumentation: monitoring devices, formwork pressure, edge-AI applications on site, durability experiments and more.",
        "If you have your own idea in this direction — even a vague one — come talk to me. The best theses start as 'would it be crazy if…'.",
      ],
      requirements: [
        { label: "Regular check-ins", detail: "short consultation at least once every 3 weeks" },
        { label: "Work plan", detail: "agreed milestone plan within the first month" },
        { label: "Draft deadline", detail: "full draft at least 3 weeks before submission — no exceptions, for your own good" },
      ],
      grading: [
        { label: "Thesis quality", pct: 70 },
        { label: "Process & independence", pct: 30 },
      ],
      homework: [],
      materials: [
        { label: "Open thesis topics", type: "pdf", url: "#" },
        { label: "Thesis formatting guide", type: "link", url: "#" },
        { label: "How to write to me (and get a fast reply)", type: "link", url: "#" },
      ],
    },
  ],
};
