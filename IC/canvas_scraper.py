#!/usr/bin/env python3
"""
UTS Canvas scraper using session cookie authentication.

Usage:
    1. Log into canvas.uts.edu.au in your browser (complete Okta 2FA)
    2. Open DevTools → Application → Cookies → canvas.uts.edu.au
    3. Copy the value of 'canvas_session'
    4. Set it in CANVAS_SESSION below (or pass via env var CANVAS_SESSION)
    5. Run: python canvas_scraper.py
"""

import os
import json
import time
import requests
from pathlib import Path

CANVAS_SESSION = os.environ.get("CANVAS_SESSION", "YOUR_COOKIE_HERE")
BASE_URL = "https://canvas.uts.edu.au/api/v1"
OUTPUT_DIR = Path("canvas_output")


def make_session():
    s = requests.Session()
    s.cookies.set("canvas_session", CANVAS_SESSION, domain="canvas.uts.edu.au")
    s.headers.update({"User-Agent": "Mozilla/5.0"})
    return s


def paginate(session, url, params=None):
    """Fetch all pages from a paginated Canvas API endpoint."""
    results = []
    params = {**(params or {}), "per_page": 100}
    while url:
        r = session.get(url, params=params)
        r.raise_for_status()
        results.extend(r.json())
        # Parse Link header for next page
        link_header = r.headers.get("Link", "")
        url = None
        params = None  # already in URL after first request
        for part in link_header.split(","):
            if 'rel="next"' in part:
                url = part.split(";")[0].strip().strip("<>")
                break
        time.sleep(0.5)
    return results


def scrape_course(session, course):
    course_id = course["id"]
    name = course.get("name", f"course_{course_id}")
    print(f"  Scraping: {name}")

    data = {"course": course}

    endpoints = {
        "modules":       f"{BASE_URL}/courses/{course_id}/modules",
        "assignments":   f"{BASE_URL}/courses/{course_id}/assignments",
        "files":         f"{BASE_URL}/courses/{course_id}/files",
        "pages":         f"{BASE_URL}/courses/{course_id}/pages",
        "announcements": f"{BASE_URL}/courses/{course_id}/discussion_topics?only_announcements=true",
    }

    for key, url in endpoints.items():
        try:
            data[key] = paginate(session, url)
            print(f"    {key}: {len(data[key])} items")
        except requests.HTTPError as e:
            print(f"    {key}: ERROR {e.response.status_code}")
            data[key] = []

    return data


def save_course(course_data):
    course = course_data["course"]
    course_id = course["id"]
    # Sanitize name for filesystem
    name = course.get("name", f"course_{course_id}")
    safe_name = "".join(c if c.isalnum() or c in " -_" else "_" for c in name).strip()
    safe_name = f"{course_id}_{safe_name}"

    out_path = OUTPUT_DIR / f"{safe_name}.json"
    out_path.write_text(json.dumps(course_data, indent=2))
    print(f"    Saved → {out_path}")

    # Also write a brief markdown summary
    md_lines = [f"# {name}\n"]
    for section in ["modules", "assignments", "files", "pages", "announcements"]:
        items = course_data.get(section, [])
        md_lines.append(f"## {section.title()} ({len(items)})\n")
        for item in items[:20]:  # first 20 per section
            title = item.get("name") or item.get("title") or item.get("display_name") or str(item.get("id"))
            md_lines.append(f"- {title}")
        if len(items) > 20:
            md_lines.append(f"- ... and {len(items) - 20} more")
        md_lines.append("")

    md_path = OUTPUT_DIR / f"{safe_name}.md"
    md_path.write_text("\n".join(md_lines))


def main():
    if CANVAS_SESSION == "YOUR_COOKIE_HERE":
        print("ERROR: Set your canvas_session cookie value in the script or via:")
        print("  export CANVAS_SESSION='your_cookie_value'")
        return

    OUTPUT_DIR.mkdir(exist_ok=True)
    session = make_session()

    print("Fetching courses...")
    try:
        courses = paginate(session, f"{BASE_URL}/courses", {"enrollment_state": "active"})
    except requests.HTTPError as e:
        if e.response.status_code in (401, 403):
            print("AUTH FAILED — cookie may be expired. Re-grab canvas_session from DevTools.")
        else:
            print(f"HTTP error: {e}")
        return

    print(f"Found {len(courses)} courses\n")

    for course in courses:
        if "name" not in course:
            continue  # skip skeleton entries
        course_data = scrape_course(session, course)
        save_course(course_data)
        print()

    print(f"Done. Output in: {OUTPUT_DIR.resolve()}")


if __name__ == "__main__":
    main()
