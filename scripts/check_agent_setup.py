#!/usr/bin/env python3
"""Compatibility entry point for the installed Agent preflight command."""

from ylhb_llm.check_agent_setup import main


if __name__ == '__main__':
    raise SystemExit(main())
