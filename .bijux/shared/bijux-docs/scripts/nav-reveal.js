(function () {
  const shell = (window.bijuxShell = window.bijuxShell || {});
  let viewportRevealBound = false;

  function centerLinkInScrollableTabs(link) {
    if (!link) {
      return;
    }

    const list = link.closest(".bijux-tabs__list");
    if (!list) {
      link.scrollIntoView({ block: "nearest", inline: "center" });
      return;
    }

    const listRect = list.getBoundingClientRect();
    const linkRect = link.getBoundingClientRect();
    const target =
      list.scrollLeft + (linkRect.left - listRect.left) - listRect.width / 2 + linkRect.width / 2;

    list.scrollTo({
      left: Math.max(0, target),
      behavior: "auto",
    });
  }

  function revealActiveNavigationTarget() {
    const activeHubLink = document.querySelector(
      ".bijux-hub-strip .bijux-tabs__item--active a"
    );
    const activeSiteLink = document.querySelector(
      ".bijux-site-tabs .bijux-tabs__item--active a"
    );
    const activeDetailLink = document.querySelector(
      "[data-bijux-detail-strip]:not([hidden]) .bijux-tabs__item--active a"
    );
    const activeCourseLink = document.querySelector(
      "[data-bijux-course-strip]:not([hidden]) .bijux-tabs__item--active a"
    );
    const activeSidebarLink = document.querySelector(
      ".md-sidebar--primary .md-nav__link--active"
    );

    centerLinkInScrollableTabs(activeHubLink);
    centerLinkInScrollableTabs(activeSiteLink);
    centerLinkInScrollableTabs(activeDetailLink);
    centerLinkInScrollableTabs(activeCourseLink);
    activeSidebarLink?.scrollIntoView({ block: "nearest", inline: "nearest" });
  }

  function revealAfterLayoutSettles() {
    revealActiveNavigationTarget();
    window.setTimeout(revealActiveNavigationTarget, 120);
    window.setTimeout(revealActiveNavigationTarget, 320);
  }

  function runDesktopNavigationSync() {
    revealAfterLayoutSettles();
  }

  function runPhoneNavigationSync() {
    bindViewportReveal();
    bindMobileDynamicTree();
    revealMobileDrawerContext();
  }

  function resolveViewportMode() {
    if (window.bijuxViewportProfile && typeof window.bijuxViewportProfile.current === "function") {
      return window.bijuxViewportProfile.current();
    }

    const fallbackWidth =
      window.visualViewport && typeof window.visualViewport.width === "number"
        ? window.visualViewport.width
        : window.innerWidth;

    if (
      window.bijuxViewportProfile &&
      typeof window.bijuxViewportProfile.classifyWidth === "function"
    ) {
      return window.bijuxViewportProfile.classifyWidth(fallbackWidth);
    }

    if (typeof window.matchMedia === "function" && window.matchMedia("(max-width: 47.9375em)").matches) {
      return "phone";
    }

    return "normal";
  }

  function directChildren(parent, selector) {
    if (!parent) {
      return [];
    }
    return Array.from(parent.children).filter((child) => child.matches(selector));
  }

  function normalizeTreePath(path) {
    const navState = shell.navState;
    if (navState && typeof navState.normalizePath === "function") {
      return navState.normalizePath(path || "/");
    }

    if (!path) {
      return "/";
    }

    const normalized = String(path).replace(/\/+$/, "");
    return normalized || "/";
  }

  function normalizeCurrentTreePath() {
    const navState = shell.navState;
    if (navState && typeof navState.normalizeCurrentPath === "function") {
      return navState.normalizeCurrentPath(window.location.pathname);
    }
    return normalizeTreePath(window.location.pathname);
  }

  function readTreeNode(nodeElement) {
    const path = normalizeTreePath(nodeElement.dataset.bijuxNodePath || "/");
    const url = nodeElement.dataset.bijuxNodeUrl || "";
    const title = nodeElement.dataset.bijuxNodeTitle || nodeElement.textContent?.trim() || "Untitled";
    const childList = directChildren(nodeElement, "ul.bijux-mobile-tree__children")[0] || null;
    const childNodes = childList
      ? directChildren(childList, "li.bijux-mobile-tree__node").map(readTreeNode)
      : [];

    return {
      title,
      path,
      url,
      children: childNodes,
    };
  }

  function dedupeNodes(nodes) {
    const uniqueNodes = [];
    const seenPaths = new Set();

    for (const node of nodes) {
      const key = node.path || node.url || node.title;
      if (!key || seenPaths.has(key)) {
        continue;
      }
      seenPaths.add(key);
      uniqueNodes.push(node);
    }

    return uniqueNodes;
  }

  function effectiveChildren(node) {
    let children = Array.isArray(node?.children) ? [...node.children] : [];
    if (!children.length) {
      return children;
    }

    const first = children[0];
    if (first && first.path === node.path && Array.isArray(first.children) && first.children.length) {
      children = first.children;
    }

    return dedupeNodes(children.filter((child) => child.path !== node.path));
  }

  function rootTreeNodes(treeNodes, isHubSite) {
    const roots = dedupeNodes(treeNodes);
    if (isHubSite) {
      return roots;
    }

    const directories = roots.filter((node) => effectiveChildren(node).length > 0);
    if (directories.length) {
      return directories;
    }

    return roots;
  }

  function findTrail(nodes, targetPath, trail = []) {
    for (const node of nodes) {
      const nextTrail = [...trail, node];
      const childNodes = effectiveChildren(node);
      const isExact = targetPath === node.path;
      const isDescendant = node.path !== "/" && targetPath.startsWith(`${node.path}/`);

      if (isExact) {
        return nextTrail;
      }

      if (isDescendant && childNodes.length) {
        const childTrail = findTrail(childNodes, targetPath, nextTrail);
        if (childTrail) {
          return childTrail;
        }
        return nextTrail;
      }
    }

    return null;
  }

  function bindMobileDynamicTree() {
    const nav = document.querySelector(".md-sidebar--primary .bijux-nav--mobile[data-bijux-nav-variant='mobile']");
    if (!nav || nav.dataset.bijuxMobileDynamicBound === "true") {
      return;
    }

    const dynamicSection = nav.querySelector("[data-bijux-mobile-dynamic]");
    const dynamicList = nav.querySelector("[data-bijux-mobile-dynamic-list]");
    const dynamicTitle = nav.querySelector("[data-bijux-mobile-nav-title]");
    const backButton = nav.querySelector("[data-bijux-mobile-nav-back]");
    const forwardButton = nav.querySelector("[data-bijux-mobile-nav-forward]");
    const treeSource = nav.querySelector("[data-bijux-mobile-tree-source] > ul.bijux-mobile-tree__list");

    if (!dynamicSection || !dynamicList || !dynamicTitle || !backButton || !forwardButton || !treeSource) {
      return;
    }

    const rootElements = directChildren(treeSource, "li.bijux-mobile-tree__node");
    if (!rootElements.length) {
      return;
    }

    const treeNodes = rootElements.map(readTreeNode);
    const isHubSite = nav.dataset.bijuxHubSite === "true";
    const rootNodes = rootTreeNodes(treeNodes, isHubSite);
    if (!rootNodes.length) {
      return;
    }

    nav.dataset.bijuxMobileDynamicBound = "true";
    const backStack = [];
    const forwardStack = [];
    let currentState = {
      title: "Navigation",
      nodes: rootNodes,
    };

    function setControlsState() {
      backButton.disabled = backStack.length === 0;
      forwardButton.disabled = forwardStack.length === 0;
    }

    function markActiveForNode(link, node, hasChildren) {
      const currentPath = normalizeCurrentTreePath();
      const isExact = currentPath === node.path;
      const isNested = node.path !== "/" && currentPath.startsWith(`${node.path}/`);
      if (isExact || (!hasChildren && isNested)) {
        link.setAttribute("aria-current", "page");
        link.closest(".md-nav__item")?.classList.add("md-nav__item--active");
      }
    }

    function renderCurrentState() {
      dynamicTitle.textContent = currentState.title || "Navigation";
      dynamicList.textContent = "";

      for (const node of currentState.nodes) {
        const item = document.createElement("li");
        item.className = "md-nav__item";

        const link = document.createElement("a");
        link.className = "md-nav__link";
        link.href = node.url || "#";
        link.textContent = node.title;

        const childNodes = effectiveChildren(node);
        const hasChildren = childNodes.length > 0;
        if (hasChildren) {
          link.dataset.bijuxMobileDrill = "true";
        }

        markActiveForNode(link, node, hasChildren);

        link.addEventListener("click", (event) => {
          const plainPrimaryClick =
            event.button === 0 &&
            !event.metaKey &&
            !event.ctrlKey &&
            !event.shiftKey &&
            !event.altKey;

          if (!hasChildren || !plainPrimaryClick) {
            return;
          }

          event.preventDefault();
          backStack.push(currentState);
          forwardStack.length = 0;
          currentState = {
            title: node.title,
            nodes: childNodes,
          };
          renderCurrentState();
        });

        item.appendChild(link);
        dynamicList.appendChild(item);
      }

      setControlsState();
    }

    backButton.addEventListener("click", () => {
      if (!backStack.length) {
        return;
      }
      forwardStack.push(currentState);
      currentState = backStack.pop();
      renderCurrentState();
    });

    forwardButton.addEventListener("click", () => {
      if (!forwardStack.length) {
        return;
      }
      backStack.push(currentState);
      currentState = forwardStack.pop();
      renderCurrentState();
    });

    const currentPath = normalizeCurrentTreePath();
    const trail = findTrail(rootNodes, currentPath);
    if (trail && trail.length) {
      for (const node of trail) {
        const childNodes = effectiveChildren(node);
        if (!childNodes.length) {
          break;
        }
        backStack.push(currentState);
        currentState = {
          title: node.title,
          nodes: childNodes,
        };
      }
    }

    renderCurrentState();
    dynamicSection.hidden = false;
    nav.dataset.bijuxMobileDynamicReady = "true";
  }

  function revealMobileDrawerContext() {
    const viewportMode = resolveViewportMode();

    if (viewportMode !== "phone") {
      return;
    }

    const activeMobileLink = document.querySelector(
      ".md-sidebar--primary .bijux-nav--mobile .md-nav__link--active, " +
        ".md-sidebar--primary .bijux-nav--mobile .md-nav__item--active > .md-nav__container > .md-nav__link, " +
        ".md-sidebar--primary .bijux-nav--mobile .md-nav__item--active > .md-nav__link"
    );

    activeMobileLink?.scrollIntoView({
      behavior: "auto",
      block: "center",
      inline: "nearest",
    });
  }

  function bindMobileDrawerReveal() {
    bindViewportReveal();
    const drawerToggle = document.querySelector("#__drawer");
    if (!drawerToggle || drawerToggle.dataset.bijuxRevealBound === "true") {
      return;
    }

    drawerToggle.dataset.bijuxRevealBound = "true";
    drawerToggle.addEventListener("change", () => {
      if (!drawerToggle.checked) {
        return;
      }

      window.setTimeout(() => {
        revealMobileDrawerContext();
      }, 180);
    });
  }

  function bindViewportReveal() {
    if (viewportRevealBound) {
      return;
    }
    viewportRevealBound = true;
    window.addEventListener("bijux:viewport-change", () => {
      revealMobileDrawerContext();
    });
  }

  shell.navReveal = {
    revealActiveNavigationTarget,
    revealAfterLayoutSettles,
    runDesktopNavigationSync,
    runPhoneNavigationSync,
    revealMobileDrawerContext,
    bindMobileDrawerReveal,
    bindMobileDynamicTree,
    bindViewportReveal,
  };
})();
