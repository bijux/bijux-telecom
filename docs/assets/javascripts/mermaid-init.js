window.mermaidConfig = {
  startOnLoad: false,
  securityLevel: "loose",
};

function activeMermaidTheme() {
  const scheme = document.body?.getAttribute("data-md-color-scheme") || "default";
  return scheme === "slate" ? "dark" : "default";
}

function normalizeMermaidBlocks() {
  // Normalize superfences output (<pre class="mermaid"><code>...</code></pre>)
  // into <div class="mermaid">...</div> so Mermaid receives raw diagram text.
  const preNodes = document.querySelectorAll("pre.mermaid");
  preNodes.forEach((pre) => {
    const code = pre.querySelector("code");
    if (!code) {
      return;
    }

    const div = document.createElement("div");
    div.className = "mermaid";
    div.textContent = code.textContent || "";
    pre.replaceWith(div);
  });
}

function prepareMermaidNodesForRerender() {
  const nodes = document.querySelectorAll("div.mermaid");
  nodes.forEach((node) => {
    const source = node.dataset.bijuxMermaidSource;
    if (!source) {
      return;
    }
    node.removeAttribute("data-processed");
    node.textContent = source;
  });
}

function renderMermaidDiagrams() {
  if (typeof mermaid === "undefined") {
    return;
  }

  mermaid.initialize({
    ...window.mermaidConfig,
    theme: activeMermaidTheme(),
  });

  normalizeMermaidBlocks();
  const nodes = document.querySelectorAll("div.mermaid");
  if (!nodes.length) {
    return;
  }

  // Persist original Mermaid definitions once, before Mermaid mutates the node.
  // Never infer source back from rendered SVG text.
  for (const node of nodes) {
    if (node.dataset.bijuxMermaidSource) {
      continue;
    }
    if (node.querySelector("svg")) {
      continue;
    }
    node.dataset.bijuxMermaidSource = node.textContent || "";
  }

  mermaid.run({ nodes });
}

function captureScrollPosition() {
  return {
    x: window.scrollX || window.pageXOffset || 0,
    y: window.scrollY || window.pageYOffset || 0,
  };
}

function restoreScrollPosition(position) {
  if (!position) {
    return;
  }
  window.scrollTo(position.x, position.y);
}

document$.subscribe(() => {
  renderMermaidDiagrams();

  if (window.__bijuxMermaidThemeBound === true) {
    return;
  }

  window.__bijuxMermaidThemeBound = true;
  window.addEventListener("bijux:theme-change", (event) => {
    const targetScroll = event?.detail?.scroll || captureScrollPosition();
    prepareMermaidNodesForRerender();
    renderMermaidDiagrams();
    restoreScrollPosition(targetScroll);
    requestAnimationFrame(() => restoreScrollPosition(targetScroll));
    setTimeout(() => restoreScrollPosition(targetScroll), 80);
  });
});
