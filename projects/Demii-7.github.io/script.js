const menuToggle = document.querySelector('.menu-toggle');
const navMenu = document.querySelector('.nav-links');
const navLinks = document.querySelectorAll('.nav-links a[href^="#"]');
const currentYear = document.querySelector('#current-year');
const mobileMenuBreakpoint = window.matchMedia('(max-width: 1024px)');

if (currentYear) {
  currentYear.textContent = String(new Date().getFullYear());
}

if (menuToggle && navMenu) {
  const closeMenu = () => {
    navMenu.classList.remove('is-open');
    menuToggle.setAttribute('aria-expanded', 'false');
    menuToggle.setAttribute('aria-label', 'Open navigation menu');
    document.body.classList.remove('menu-open');
  };

  const openMenu = () => {
    navMenu.classList.add('is-open');
    menuToggle.setAttribute('aria-expanded', 'true');
    menuToggle.setAttribute('aria-label', 'Close navigation menu');
    document.body.classList.add('menu-open');
  };

  menuToggle.addEventListener('click', () => {
    const isOpen = navMenu.classList.contains('is-open');

    if (isOpen) {
      closeMenu();
      return;
    }

    openMenu();
  });

  navLinks.forEach((link) => {
    link.addEventListener('click', () => {
      closeMenu();
    });
  });

  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') {
      closeMenu();
    }
  });

  document.addEventListener('click', (event) => {
    if (!mobileMenuBreakpoint.matches || !navMenu.classList.contains('is-open')) {
      return;
    }

    if (navMenu.contains(event.target) || menuToggle.contains(event.target)) {
      return;
    }

    closeMenu();
  });

  const handleBreakpointChange = (event) => {
    if (!event.matches) {
      closeMenu();
    }
  };

  if (typeof mobileMenuBreakpoint.addEventListener === 'function') {
    mobileMenuBreakpoint.addEventListener('change', handleBreakpointChange);
  } else if (typeof mobileMenuBreakpoint.addListener === 'function') {
    mobileMenuBreakpoint.addListener(handleBreakpointChange);
  }
}
