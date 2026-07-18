(function () {
  // Compatibility entrypoint: shell navigation logic lives in .bijux/shared/bijux-docs/scripts/*
  // and is wired through bootstrap.js after synchronization into docs/assets/javascripts/shell/*.
  if (window.bijuxShell?.bootstrap?.ensureBound) {
    window.bijuxShell.bootstrap.ensureBound();
  }
})();
