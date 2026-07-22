# Field indices, in top-to-bottom focus order.
F_MODE, F_RES, F_DELAY, F_COUNT, F_SEND, F_CANCEL, F_EXIT = range(7)
NFIELDS = 7

# Returned by the form when the user hits Exit -- distinct from a config dict (Send)
# and from None (Cancel = listen passively). Callers quit the program on this.
QUIT = "quit"

MODE_LABELS = ["Continuous", "Specific count"]


def _res_label(res_names, res_dims, i):
    """"QVGA (320x240)"-style label; modes whose name is already the size show bare."""
    name = res_names[i]
    dims = res_dims.get(name)
    return f"{name} ({dims})" if dims else name


def run_config_form(stdscr, res_names, res_dims, *, mode_default=0, res_default=8,
                    delay_default=5, count_default=1,
                    delay_max=32767, count_max=4095):
    """Run the config form loop on an already-initialised curses screen `stdscr`,
    returning dict(mode, res, delay, count) on Send or None on cancel (Cancel button,
    'q', or Esc). This is the reusable core: the standalone `config_tui` wraps it in
    curses.wrapper, and the status Dashboard calls it modally on its own screen.

    The caller is responsible for cbreak/noecho; here we only set the form's own
    input expectations (cursor hidden, keypad on, blocking getch)."""
    import curses

    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.nodelay(False)                 # the form wants blocking input
    try:
        curses.set_escdelay(25)           # snappy Esc-to-cancel (default is ~1s)
    except (AttributeError, curses.error):
        pass
    has_color = curses.has_colors()
    if has_color:
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)   # focused/selected
        curses.init_pair(2, curses.COLOR_CYAN, -1)                   # accents
        curses.init_pair(3, curses.COLOR_YELLOW, -1)                 # config preview
        curses.init_pair(4, curses.COLOR_RED, -1)                    # exit button

    def A(pair, extra=0):
        return (curses.color_pair(pair) if has_color else 0) | extra

    state = {
        "mode": mode_default,
        "res": res_default,
        "delay": str(delay_default),
        "count": str(count_default),
        "focus": F_RES,      # start on the resolution list -- the main choice
        "result": None,
    }
    while True:
        _draw(stdscr, state, res_names, res_dims, delay_max, count_max, A)
        key = stdscr.getch()
        if _handle_key(key, state, res_names, delay_max, count_max):
            return state["result"]      # submit or cancel set state["result"]


def config_tui(res_names, res_dims, **kw):
    """Drive the config form in a fresh curses session and return its selections dict
    (or None if cancelled). Runs under curses.wrapper so the terminal is restored on
    any exit path, including exceptions. May raise if the terminal can't enter curses
    mode -- callers that want a fallback should catch that."""
    import curses
    return curses.wrapper(
        lambda stdscr: run_config_form(stdscr, res_names, res_dims, **kw))


def _num_focus(state):
    """The state key of the focused numeric field, or None if none is focused."""
    return {F_DELAY: "delay", F_COUNT: "count"}.get(state["focus"])


def _handle_key(key, state, res_names, delay_max, count_max):
    """Apply one keypress to `state`. Return True when the form should exit."""
    import curses

    focus = state["focus"]
    count_active = state["mode"] == 1

    # Global cancel.
    if key in (27, ord("q"), ord("Q")):      # Esc / q
        state["result"] = None
        return True

    # Focus movement. Down/Tab and Up/Shift-Tab walk the field list, skipping the
    # Count field while it's inactive (continuous mode).
    def step_focus(delta):
        f = state["focus"]
        while True:
            f = (f + delta) % NFIELDS
            if f == F_COUNT and not count_active:
                continue
            break
        state["focus"] = f

    # Tab always cycles between fields.
    if key == ord("\t"):
        step_focus(1); return False
    if key == curses.KEY_BTAB:
        step_focus(-1); return False

    # Up/Down move the highlight *within* the resolution list while it's focused
    # (the natural gesture for a vertical list), stepping to the neighbouring field
    # only once you run off the top/bottom. On every other field they move focus.
    if key == curses.KEY_DOWN:
        if focus == F_RES and state["res"] < len(res_names) - 1:
            state["res"] += 1
        else:
            step_focus(1)
        return False
    if key == curses.KEY_UP:
        if focus == F_RES and state["res"] > 0:
            state["res"] -= 1
        else:
            step_focus(-1)
        return False

    # Numeric entry (delay / count).
    nkey = _num_focus(state)
    if nkey is not None:
        if ord("0") <= key <= ord("9"):
            new = (state[nkey] + chr(key)).lstrip("0") or "0"
            cap = delay_max if nkey == "delay" else count_max
            if int(new) <= cap:
                state[nkey] = new
            return False
        if key in (curses.KEY_BACKSPACE, 127, 8):
            state[nkey] = state[nkey][:-1] or "0"
            return False

    # Left/right and space act on the focused widget.
    if focus == F_MODE:
        if key in (curses.KEY_LEFT, curses.KEY_RIGHT, ord(" ")):
            state["mode"] ^= 1
            return False
    elif focus == F_RES:
        if key == curses.KEY_LEFT:
            state["res"] = (state["res"] - 1) % len(res_names); return False
        if key == curses.KEY_RIGHT:
            state["res"] = (state["res"] + 1) % len(res_names); return False

    # Activation: Enter/Space on the buttons.
    if key in (curses.KEY_ENTER, 10, 13):
        if focus == F_SEND:
            state["result"] = dict(mode=state["mode"], res=state["res"],
                                   delay=int(state["delay"] or 0),
                                   count=int(state["count"] or 0))
            return True
        if focus == F_CANCEL:
            state["result"] = None
            return True
        if focus == F_EXIT:
            state["result"] = QUIT
            return True
        if focus == F_MODE:
            state["mode"] ^= 1
    return False


def _draw(stdscr, state, res_names, res_dims, delay_max, count_max, A):
    """Render the whole form. Every addstr is guarded so a too-small terminal just
    clips instead of raising."""
    import curses

    stdscr.erase()
    h, w = stdscr.getmaxyx()
    focus = state["focus"]
    count_active = state["mode"] == 1

    def put(y, x, s, attr=0):
        if 0 <= y < h and x >= 0:
            try:
                stdscr.addnstr(y, x, s, max(0, w - x - 1), attr)
            except curses.error:
                pass

    box_w = min(w - 2, 60)
    # Title bar.
    put(0, 2, "slow-loras · new capture", A(2, curses.A_BOLD))
    put(1, 2, "─" * (box_w - 2), A(2))

    # --- Mode ---------------------------------------------------------------
    y = 3
    label_attr = A(1, curses.A_BOLD) if focus == F_MODE else curses.A_BOLD
    put(y, 2, " Mode ", label_attr)
    x = 10
    for i, lab in enumerate(MODE_LABELS):
        mark = "◉" if state["mode"] == i else "◯"     # filled / empty radio
        chunk = f"{mark} {lab}"
        sel = state["mode"] == i
        put(y, x, chunk, A(2, curses.A_BOLD) if sel else 0)
        x += len(chunk) + 3

    # --- Resolution ---------------------------------------------------------
    y = 5
    put(y, 2, " Resolution ", A(1, curses.A_BOLD) if focus == F_RES else curses.A_BOLD)
    # Scrolling viewport so a short terminal still fits.
    list_top = y + 1
    # Reserve 8 rows below the list: blank, delay, count, blank, buttons, blank,
    # preview, hint. The list scrolls if it can't show all rows in what's left.
    avail = max(3, h - list_top - 8)
    view = min(len(res_names), avail)
    sel = state["res"]
    start = max(0, min(sel - view // 2, len(res_names) - view))
    for row in range(view):
        i = start + row
        selected = i == sel
        ptr = "▸ " if selected else "  "
        line = f"{ptr}{i:2d}) {_res_label(res_names, res_dims, i)}"
        attr = A(1) if (selected and focus == F_RES) else (A(2) if selected else 0)
        put(list_top + row, 4, line, attr)
    if start > 0:
        put(list_top, 2, "↑")
    if start + view < len(res_names):
        put(list_top + view - 1, 2, "↓")

    # --- Delay / Count (below the list) -------------------------------------
    y = list_top + view + 1

    def field(fy, label, key, active=True):
        focused = focus == {"delay": F_DELAY, "count": F_COUNT}[key]
        lab_attr = A(1, curses.A_BOLD) if focused else curses.A_BOLD
        if not active:
            lab_attr = curses.A_DIM
        put(fy, 2, f" {label} ", lab_attr)
        box_attr = A(1) if focused else (curses.A_DIM if not active else A(2))
        val = state[key] if active else "-"
        put(fy, 16, f"[ {val:>6} ]", box_attr)

    field(y, "Delay (s) ", "delay")
    field(y + 1, "Count     ", "count", active=count_active)

    # --- Buttons ------------------------------------------------------------
    by = y + 3
    send_attr = A(1, curses.A_BOLD) if focus == F_SEND else A(2, curses.A_BOLD)
    cancel_attr = A(1, curses.A_BOLD) if focus == F_CANCEL else 0
    exit_attr = A(1, curses.A_BOLD) if focus == F_EXIT else A(4, curses.A_BOLD)
    put(by, 6, "  Send  ", send_attr)
    put(by, 18, " Cancel ", cancel_attr)
    put(by, 30, "  Exit  ", exit_attr)

    # --- Live config preview + key hints ------------------------------------
    cnt = int(state["count"] or 0) if count_active else 0
    cfg = _preview_cfg(state["mode"], state["res"], int(state["delay"] or 0), cnt)
    mode_txt = "continuous" if state["mode"] == 0 else "count"
    summary = (f"cfg 0x{cfg:08X} · mode={mode_txt} res={res_names[state['res']]} "
               f"delay={int(state['delay'] or 0)}s count={cnt}")
    put(by + 2, 2, summary, A(3))
    hint = "↑↓ select · Tab next field · space toggle · 0-9 type · Enter=activate · q/Esc cancel"
    put(by + 3, 2, hint, curses.A_DIM)

    stdscr.refresh()


def _preview_cfg(mode, res, delay, count):
    """Same bit layout as lora_serial.pack_config -- kept local so the preview never
    imports back into lora_serial (would be circular). Display only."""
    return ((mode & 0x1) | ((res & 0xF) << 1) | ((delay & 0x7FFF) << 5)
            | ((count & 0xFFF) << 20)) & 0xFFFFFFFF
