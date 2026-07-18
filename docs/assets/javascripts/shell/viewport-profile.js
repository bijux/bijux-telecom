(function () {
  // Breakpoint contract:
  // - phone: < 48em
  // - normal: 48em to 76.2344em
  // - desktop: > 76.2344em and < 120em
  // - wide: >= 120em
  const PHONE_MAX_MEDIA = "(max-width: 47.9375em)";
  const NORMAL_MAX_MEDIA = "(max-width: 76.2344em)";
  const WIDE_MIN_MEDIA = "(min-width: 120em)";
  const VIEWPORT_PROFILE_SOURCE = ".bijux/shared/bijux-docs/scripts/viewport-profile.js";
  const VIEWPORT_PROFILE_REVISION = "2026-04-17";
  const VIEWPORT_PROFILES = Object.freeze({
    PHONE: "phone",
    NORMAL: "normal",
    DESKTOP: "desktop",
    WIDE: "wide",
  });
  const PROFILE_BOUNDARIES_EM = Object.freeze({
    PHONE_MAX: 47.9375,
    NORMAL_MAX: 76.2344,
    WIDE_MIN: 120,
  });
  const MEDIA_QUERY_BASE_FONT_PX = 16;
  const REFERENCE_WIDTHS = Object.freeze({
    phone390: 390,
    normal768: 768,
    normal1024: 1024,
    desktop1280: 1280,
    wide1920: 1920,
  });
  const REFERENCE_PROFILE_EXPECTATIONS = Object.freeze({
    390: VIEWPORT_PROFILES.PHONE,
    768: VIEWPORT_PROFILES.NORMAL,
    1024: VIEWPORT_PROFILES.NORMAL,
    1280: VIEWPORT_PROFILES.DESKTOP,
    1920: VIEWPORT_PROFILES.WIDE,
  });

  function mediaMatches(query) {
    return typeof window.matchMedia === "function" && window.matchMedia(query).matches;
  }

  function toPixelsFromEm(em) {
    // Media-query em units map to the browser's initial font size (16px in our shell assumptions),
    // not the potentially customized runtime <html> computed font-size.
    return em * MEDIA_QUERY_BASE_FONT_PX;
  }

  function normalizeViewportWidth(width) {
    if (!Number.isFinite(width) || width <= 0) {
      return Number.NaN;
    }
    return Math.round(width * 100) / 100;
  }

  function currentViewportWidth() {
    const documentWidth = normalizeViewportWidth(document.documentElement?.clientWidth);
    const visualViewportWidth = normalizeViewportWidth(window.visualViewport?.width);
    const innerWidth = normalizeViewportWidth(window.innerWidth);

    if (Number.isFinite(documentWidth) && Number.isFinite(visualViewportWidth)) {
      // On mobile browsers during URL-bar transitions, the two values can diverge briefly.
      // Using the narrower width avoids classifying as a larger layout band too early.
      return Math.min(documentWidth, visualViewportWidth);
    }

    if (Number.isFinite(documentWidth)) {
      return documentWidth;
    }

    if (Number.isFinite(visualViewportWidth)) {
      return visualViewportWidth;
    }

    if (Number.isFinite(innerWidth)) {
      return innerWidth;
    }

    return Number.NaN;
  }

  function classifyViewportWidth(width) {
    const normalizedWidth = normalizeViewportWidth(width);
    if (!Number.isFinite(normalizedWidth)) {
      return VIEWPORT_PROFILES.DESKTOP;
    }
    if (normalizedWidth <= toPixelsFromEm(PROFILE_BOUNDARIES_EM.PHONE_MAX)) {
      return VIEWPORT_PROFILES.PHONE;
    }
    if (normalizedWidth >= toPixelsFromEm(PROFILE_BOUNDARIES_EM.WIDE_MIN)) {
      return VIEWPORT_PROFILES.WIDE;
    }
    if (normalizedWidth <= toPixelsFromEm(PROFILE_BOUNDARIES_EM.NORMAL_MAX)) {
      return VIEWPORT_PROFILES.NORMAL;
    }
    return VIEWPORT_PROFILES.DESKTOP;
  }

  function resolveReferenceProfiles() {
    return {
      390: classifyViewportWidth(REFERENCE_WIDTHS.phone390),
      768: classifyViewportWidth(REFERENCE_WIDTHS.normal768),
      1024: classifyViewportWidth(REFERENCE_WIDTHS.normal1024),
      1280: classifyViewportWidth(REFERENCE_WIDTHS.desktop1280),
      1920: classifyViewportWidth(REFERENCE_WIDTHS.wide1920),
    };
  }

  function verifyReferenceContract() {
    const actual = resolveReferenceProfiles();
    const mismatches = Object.keys(REFERENCE_PROFILE_EXPECTATIONS).reduce((result, widthKey) => {
      const expectedProfile = REFERENCE_PROFILE_EXPECTATIONS[widthKey];
      const actualProfile = actual[widthKey];
      if (actualProfile !== expectedProfile) {
        result[widthKey] = {
          expected: expectedProfile,
          actual: actualProfile,
        };
      }
      return result;
    }, {});
    return {
      ok: Object.keys(mismatches).length === 0,
      expected: { ...REFERENCE_PROFILE_EXPECTATIONS },
      actual,
      mismatches,
    };
  }

  function resolveViewportProfile() {
    if (typeof window.matchMedia !== "function") {
      return classifyViewportWidth(currentViewportWidth());
    }

    if (mediaMatches(PHONE_MAX_MEDIA)) {
      return VIEWPORT_PROFILES.PHONE;
    }

    if (mediaMatches(WIDE_MIN_MEDIA)) {
      return VIEWPORT_PROFILES.WIDE;
    }

    if (mediaMatches(NORMAL_MAX_MEDIA)) {
      return VIEWPORT_PROFILES.NORMAL;
    }

    return VIEWPORT_PROFILES.DESKTOP;
  }

  let currentProfile = null;

  function writeViewportAttribute(target, profile) {
    if (!target || typeof target.setAttribute !== "function") {
      return;
    }
    if (target.getAttribute("data-bijux-viewport") !== profile) {
      target.setAttribute("data-bijux-viewport", profile);
    }
    if (target.getAttribute("data-bijux-viewport-source") !== VIEWPORT_PROFILE_SOURCE) {
      target.setAttribute("data-bijux-viewport-source", VIEWPORT_PROFILE_SOURCE);
    }
    if (target.getAttribute("data-bijux-viewport-revision") !== VIEWPORT_PROFILE_REVISION) {
      target.setAttribute("data-bijux-viewport-revision", VIEWPORT_PROFILE_REVISION);
    }
  }

  function applyViewportProfile() {
    const profile = resolveViewportProfile();
    const previousProfile = currentProfile;
    const width = currentViewportWidth();
    // Keep both targets in sync because CSS and JS hooks read from each in different contexts.
    writeViewportAttribute(document.documentElement, profile);
    writeViewportAttribute(document.body, profile);
    if (profile !== currentProfile) {
      window.dispatchEvent(
        new CustomEvent("bijux:viewport-change", {
          detail: {
            profile,
            previousProfile,
            width,
          },
        })
      );
      currentProfile = profile;
    }
    return profile;
  }

  function bindViewportUpdates() {
    if (window.__bijuxViewportBound === true) {
      return;
    }

    window.__bijuxViewportBound = true;

    let rafId = 0;
    let settleTimerId = 0;
    const scheduleApply = () => {
      if (rafId !== 0) {
        return;
      }
      // Coalesce rapid resize/orientation events into one profile recompute per frame.
      rafId = window.requestAnimationFrame(() => {
        rafId = 0;
        applyViewportProfile();
      });
    };
    const scheduleApplyAfterSettle = () => {
      scheduleApply();
      if (settleTimerId !== 0) {
        window.clearTimeout(settleTimerId);
      }
      // Some phone browsers update final width a moment after orientation events.
      settleTimerId = window.setTimeout(() => {
        settleTimerId = 0;
        scheduleApply();
      }, 140);
    };
    const clearPendingUpdates = () => {
      if (rafId !== 0) {
        window.cancelAnimationFrame(rafId);
        rafId = 0;
      }
      if (settleTimerId !== 0) {
        window.clearTimeout(settleTimerId);
        settleTimerId = 0;
      }
    };

    window.addEventListener("resize", scheduleApply, { passive: true });
    window.addEventListener("orientationchange", scheduleApplyAfterSettle, { passive: true });
    window.addEventListener("pageshow", scheduleApplyAfterSettle, { passive: true });
    window.addEventListener(
      "visibilitychange",
      () => {
        if (document.visibilityState === "visible") {
          scheduleApply();
        }
      },
      { passive: true }
    );
    window.addEventListener("pagehide", clearPendingUpdates, { passive: true });

    if (window.visualViewport && typeof window.visualViewport.addEventListener === "function") {
      window.visualViewport.addEventListener("resize", scheduleApply, { passive: true });
    }

    if (typeof window.matchMedia === "function") {
      [PHONE_MAX_MEDIA, NORMAL_MAX_MEDIA, WIDE_MIN_MEDIA].forEach((query) => {
        const mediaQuery = window.matchMedia(query);
        if (mediaQuery && typeof mediaQuery.addEventListener === "function") {
          mediaQuery.addEventListener("change", scheduleApply);
        } else if (mediaQuery && typeof mediaQuery.addListener === "function") {
          mediaQuery.addListener(scheduleApply);
        }
      });
    }
  }

  function init() {
    applyViewportProfile();
    bindViewportUpdates();
    const verification = verifyReferenceContract();
    if (!verification.ok && typeof console !== "undefined" && typeof console.warn === "function") {
      console.warn("[bijux][viewport-profile] Reference width contract mismatch", verification);
    }
  }

  function initWithFallback() {
    if (typeof document$ !== "undefined" && document$ && typeof document$.subscribe === "function") {
      document$.subscribe(init);
      return;
    }

    if (document.readyState === "loading") {
      document.addEventListener("DOMContentLoaded", init, { once: true });
      return;
    }

    init();
  }

  window.bijuxViewportProfile = {
    current: resolveViewportProfile,
    apply: applyViewportProfile,
    classifyWidth: classifyViewportWidth,
    media: {
      phoneMax: PHONE_MAX_MEDIA,
      normalMax: NORMAL_MAX_MEDIA,
      wideMin: WIDE_MIN_MEDIA,
    },
    source: VIEWPORT_PROFILE_SOURCE,
    revision: VIEWPORT_PROFILE_REVISION,
    signature: `${VIEWPORT_PROFILE_SOURCE}@${VIEWPORT_PROFILE_REVISION}`,
    verifyReferenceWidths: resolveReferenceProfiles,
    verifyContract: verifyReferenceContract,
    referenceExpectations: () => ({ ...REFERENCE_PROFILE_EXPECTATIONS }),
    describe: () => {
      const profile = resolveViewportProfile();
      return {
        profile,
        width: currentViewportWidth(),
        references: resolveReferenceProfiles(),
        matches: {
          phone: profile === VIEWPORT_PROFILES.PHONE,
          normalBand: profile === VIEWPORT_PROFILES.NORMAL,
          desktopBand: profile === VIEWPORT_PROFILES.DESKTOP,
          wide: profile === VIEWPORT_PROFILES.WIDE,
        },
        mediaMatches: {
          phoneMax: mediaMatches(PHONE_MAX_MEDIA),
          normalMax: mediaMatches(NORMAL_MAX_MEDIA),
          wideMin: mediaMatches(WIDE_MIN_MEDIA),
        },
        thresholdsPx: {
          phoneMax: toPixelsFromEm(PROFILE_BOUNDARIES_EM.PHONE_MAX),
          normalMax: toPixelsFromEm(PROFILE_BOUNDARIES_EM.NORMAL_MAX),
          wideMin: toPixelsFromEm(PROFILE_BOUNDARIES_EM.WIDE_MIN),
        },
      };
    },
  };

  initWithFallback();
})();
