(function () {
  const shell = (window.bijuxShell = window.bijuxShell || {});

  function runShellNavigationSync() {
    const viewportProfile =
      window.bijuxViewportProfile && typeof window.bijuxViewportProfile.current === "function"
        ? window.bijuxViewportProfile.current()
        : "normal";

    shell.detailTabs?.bindDetailSelectNavigation?.();
    shell.navReveal?.bindMobileDrawerReveal?.();

    if (viewportProfile === "phone") {
      shell.detailTabs?.runPhoneNavigationSync?.();
      shell.navReveal?.runPhoneNavigationSync?.();
      return;
    }

    shell.detailTabs?.runDesktopNavigationSync?.();
    shell.navReveal?.runDesktopNavigationSync?.();
  }

  function ensureBound() {
    if (window.__bijuxShellBootstrapBound === true) {
      return;
    }

    window.__bijuxShellBootstrapBound = true;
    document$.subscribe(runShellNavigationSync);
  }

  shell.bootstrap = {
    runShellNavigationSync,
    ensureBound,
  };

  ensureBound();
})();
