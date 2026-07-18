function bijuxIsOffsiteLink(link) {
  const href = link.getAttribute("href");
  if (!href || href.startsWith("#")) {
    return false;
  }

  let url;
  try {
    url = new URL(href, window.location.href);
  } catch {
    return false;
  }

  if (url.origin === window.location.origin) {
    return false;
  }

  return url.protocol === "http:" || url.protocol === "https:";
}

function bijuxMarkOffsiteLinks() {
  for (const link of document.querySelectorAll("a[href]")) {
    if (!bijuxIsOffsiteLink(link)) {
      link.removeAttribute("data-bijux-external-link");
      continue;
    }

    link.setAttribute("target", "_blank");
    link.setAttribute("rel", "noopener noreferrer");
    link.setAttribute("data-bijux-external-link", "true");
  }
}

document$.subscribe(() => {
  bijuxMarkOffsiteLinks();
});
