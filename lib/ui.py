"""Status UI shared by both receive paths.

Both the serial and SDR drivers report the same three things -- log lines, a live
progress bar, and (serial only) a config prompt. This module gives them one small
interface with two implementations:

  * `Dashboard` -- a full-screen curses status view (header, scrolling log, live
    progress bar, footer). Used when stdout is a real terminal. The serial path also
    opens the config form (lib.tui) modally on the same screen; the SDR path just
    streams packet status into it. It is thread-safe: the SDR decoder and rtl_sdr
    stderr-relay threads log from off the main thread, so every curses touch is
    serialised under one lock.
  * `ConsoleUI` -- the original plain behaviour (log lines to stdout, progress bar
    to stderr). Used for headless / piped / --file runs so nothing about the
    non-interactive experience changes.

`make_ui` picks one: a Dashboard when stdout is a TTY (falling back to ConsoleUI if
curses won't start), else ConsoleUI. `curses` is imported lazily inside Dashboard so
importing this module on a headless host stays free.
"""
import sys
import threading
from collections import deque

from lib import helpers


def make_ui(source, subtitle=""):
    """Return a started UI: a Dashboard when stdout is a TTY (curses permitting),
    otherwise a ConsoleUI. Caller must `close()` it (both are also context managers)."""
    if sys.stdout.isatty():
        d = Dashboard(source, subtitle)
        try:
            d.start()
            return d
        except Exception:
            try:
                d.close()
            except Exception:
                pass
    return ConsoleUI()


class ConsoleUI:
    """Plain stdout/stderr UI -- byte-for-byte the pre-TUI behaviour. `check_quit`
    is always False (Ctrl-C remains the way to interrupt) and there is no interactive
    form, so callers fall back to text prompts."""

    interactive_form = False

    def start(self):
        return self

    def close(self):
        pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    def log(self, msg):
        print(msg, flush=True)

    def status(self, msg):
        print(msg, flush=True)

    def progress(self, frac, label="receiving", detail=""):
        w = 24
        if frac is not None:
            bar = helpers.progress_bar(frac, w)
            sys.stderr.write(f"\r  {label} [{bar}] {frac * 100:3.0f}%  {detail}   ")
        else:
            sys.stderr.write(f"\r  {label} [{'.' * w}]  {detail}   ")
        sys.stderr.flush()

    def end_progress(self):
        sys.stderr.write("\n")
        sys.stderr.flush()

    def check_quit(self):
        return False


class Dashboard:
    """Curses status dashboard. Public methods are thread-safe."""

    interactive_form = True

    def __init__(self, source, subtitle=""):
        self.source = source
        self.subtitle = subtitle
        self.status_text = ""
        self.lines = deque(maxlen=1000)     # scrollback (we show the tail)
        self.prog = None                    # (frac_or_None, label, detail)
        self._lock = threading.RLock()
        self.stdscr = None
        self._active = False
        self._has_color = False

    # -- lifecycle -----------------------------------------------------------
    def start(self):
        import curses
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(True)
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        try:
            curses.set_escdelay(25)
        except (AttributeError, curses.error):
            pass
        self._has_color = curses.has_colors()
        if self._has_color:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)   # highlight
            curses.init_pair(2, curses.COLOR_CYAN, -1)                   # accents
            curses.init_pair(3, curses.COLOR_YELLOW, -1)                 # progress
            curses.init_pair(4, curses.COLOR_RED, -1)                    # warnings
            curses.init_pair(5, curses.COLOR_GREEN, -1)                  # saved/OK
        self.stdscr.nodelay(True)           # check_quit polls without blocking
        self._active = True
        self._render()
        return self

    def close(self):
        import curses
        if not self._active:
            return
        self._active = False
        try:
            curses.nocbreak()
            self.stdscr.keypad(False)
            curses.echo()
            curses.endwin()
        except curses.error:
            pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    # -- status surface ------------------------------------------------------
    def log(self, msg):
        with self._lock:
            for line in str(msg).split("\n"):
                self.lines.append(line)
            self._render()

    def status(self, msg):
        with self._lock:
            self.status_text = str(msg)
            self._render()

    def progress(self, frac, label="receiving", detail=""):
        with self._lock:
            self.prog = (frac, label, detail)
            self._render()

    def end_progress(self):
        with self._lock:
            self.prog = None
            self._render()

    def check_quit(self):
        """True if the user pressed q/Esc since last checked (non-blocking)."""
        if not self._active:
            return False
        import curses
        with self._lock:
            try:
                k = self.stdscr.getch()
            except curses.error:
                return False
        if k == curses.KEY_RESIZE:
            self._render()
            return False
        return k in (ord("q"), ord("Q"), 27)

    # -- modal config form ---------------------------------------------------
    def config_form(self, res_names, res_dims, res_default=8):
        """Open the lib.tui config form modally on this screen. Returns a selections
        dict on Send, or None on cancel. Restores the dashboard view afterwards."""
        from lib import tui
        with self._lock:
            try:
                sel = tui.run_config_form(self.stdscr, res_names, res_dims,
                                          res_default=res_default)
            finally:
                if self.stdscr is not None:
                    self.stdscr.nodelay(True)     # back to non-blocking for check_quit
            self._render()
        return sel

    # -- rendering -----------------------------------------------------------
    def _attr(self, pair, extra=0):
        import curses
        return (curses.color_pair(pair) if self._has_color else 0) | extra

    def _line_attr(self, line):
        low = line.lower()
        if any(w in low for w in ("error", "undecodable", "dropped", "no image",
                                  "no ack", "offline", "mismatch")):
            return self._attr(4)
        if any(w in low for w in ("saved", "ack ok", "received")):
            return self._attr(5)
        return 0

    def _render(self):
        if not self._active:
            return
        import curses
        scr = self.stdscr
        h, w = scr.getmaxyx()

        def put(y, x, s, attr=0):
            if 0 <= y < h and 0 <= x < w:
                try:
                    scr.addnstr(y, x, s, max(0, w - x - 1), attr)
                except curses.error:
                    pass

        scr.erase()
        # Header: title left, source right.
        title = " molasses-cam"
        put(0, 1, title, self._attr(2, curses.A_BOLD))
        src = f"{self.source}{('  ·  ' + self.subtitle) if self.subtitle else ''} "
        put(0, max(len(title) + 2, w - len(src) - 1), src, self._attr(2))
        put(1, 1, "─" * (w - 2), self._attr(2))
        if self.status_text:
            put(2, 1, self.status_text, curses.A_BOLD)

        # Bottom-up: footer hint (h-1), progress bar (h-2), divider (h-3); the log
        # fills the middle from row 4 down to h-4.
        foot_y = h - 1
        prog_y = h - 2
        div_y = h - 3
        log_top, log_bot = 4, h - 4

        tail = list(self.lines)[-(max(0, log_bot - log_top + 1)):] if log_bot >= log_top else []
        for i, line in enumerate(tail):
            put(log_top + i, 2, line, self._line_attr(line))

        put(div_y, 1, "─" * (w - 2), self._attr(2))

        if self.prog is not None:
            frac, label, detail = self.prog
            bw = max(10, min(30, w - 30))
            if frac is not None:
                fr = max(0.0, min(1.0, frac))
                fill = int(fr * bw)
                bar = "█" * fill + "░" * (bw - fill)
                put(prog_y, 2, f"{label} ", self._attr(3, curses.A_BOLD))
                put(prog_y, 2 + len(label) + 1, f"[{bar}] {fr * 100:3.0f}%  {detail}",
                    self._attr(3))
            else:
                bar = "░" * bw
                put(prog_y, 2, f"{label} [{bar}]  {detail}", self._attr(3))

        hint = ("q quit" if self.source.lower().startswith("sdr")
                else "q return to menu · Ctrl-C quit")
        put(foot_y, 3, f" {hint} ", curses.A_DIM)
        scr.refresh()
