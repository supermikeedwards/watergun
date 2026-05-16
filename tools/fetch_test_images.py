"""Fetch CC-licensed bird photos from iNaturalist for the offline validation test set.

Helper script — run once to populate test_images/. Files are gitignored, so the
photos only exist on whoever ran this script.

Sources photos under CC0, CC-BY, CC-BY-SA (excludes NC / ND).
Saves each image as test_images/{species_slug}_{n}_{photo_id}.jpg with
attribution captured in test_images/attribution.txt.
"""
from __future__ import annotations

import json
import os
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "test_images"
ATTRIB_FILE = OUT_DIR / "attribution.txt"

# (taxon_id, slug, count) — total 19 photos.
SPECIES = [
    (3048,   "wood_pigeon",  4),  # Common Wood-Pigeon — most likely in Mike's cherry tree
    (12716,  "blackbird",    3),  # Eurasian Blackbird
    (14850,  "starling",     3),  # European Starling
    (13858,  "sparrow",      3),  # House Sparrow
    (891696, "magpie",       3),  # Eurasian Magpie
    (144849, "blue_tit",     3),  # Eurasian Blue Tit
]

# CC0, CC-BY, CC-BY-SA only — no NC/ND.
LICENSES = "cc0,cc-by,cc-by-sa"
USER_AGENT = "pigeon_watergun-validation/1.0 (test set curation)"


def fetch_observations(taxon_id: int, n: int) -> list[dict]:
    """Returns up to n*3 observations to give us spare candidates after filtering."""
    params = {
        "taxon_id": str(taxon_id),
        "photo_license": LICENSES,
        "photos": "true",
        "quality_grade": "research",
        "per_page": str(min(50, n * 5)),
        "order_by": "random",
    }
    url = "https://api.inaturalist.org/v1/observations?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=15) as r:
        data = json.loads(r.read())
    return data.get("results", [])


def to_medium_url(square_url: str) -> str:
    """iNaturalist photo URLs follow {base}/{photo_id}/{size}.{ext}.
    Replace 'square' (75px) with 'medium' (500px) for a usable image size."""
    return square_url.replace("/square.", "/medium.").replace("/square?", "/medium?")


def download(url: str, dest: Path) -> bool:
    try:
        req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(req, timeout=15) as r:
            data = r.read()
        if len(data) < 5000:
            # Sanity: medium iNat photos are typically 30-200KB. <5KB = likely a 404 image.
            return False
        dest.write_bytes(data)
        return True
    except Exception as e:
        print(f"    download failed: {e}")
        return False


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    attribs = []
    saved_total = 0
    for taxon_id, slug, target in SPECIES:
        print(f"\n[{slug}] target={target}")
        try:
            obs = fetch_observations(taxon_id, target)
        except Exception as e:
            print(f"  iNat query failed: {e}")
            continue
        n_saved = 0
        for o in obs:
            if n_saved >= target:
                break
            photos = o.get("photos") or []
            if not photos:
                continue
            p = photos[0]
            photo_id = p.get("id")
            square_url = p.get("url")
            if not square_url:
                continue
            medium_url = to_medium_url(square_url)
            licence = p.get("license_code", "?")
            attribution = p.get("attribution") or f"iNaturalist photo {photo_id}"
            obs_id = o.get("id")
            obs_url = f"https://www.inaturalist.org/observations/{obs_id}"
            dest = OUT_DIR / f"{slug}_{n_saved + 1:02d}_{photo_id}.jpg"
            if dest.exists():
                print(f"  skip (exists): {dest.name}")
                n_saved += 1
                continue
            print(f"  -> {dest.name} ({licence})")
            if download(medium_url, dest):
                n_saved += 1
                saved_total += 1
                attribs.append(
                    f"{dest.name}\n"
                    f"  Photo: {medium_url}\n"
                    f"  Observation: {obs_url}\n"
                    f"  Licence: {licence}\n"
                    f"  Attribution: {attribution}\n"
                )
                time.sleep(0.3)  # be polite to the API
        print(f"  saved {n_saved}/{target}")

    # Append (don't overwrite) attribution file
    mode = "a" if ATTRIB_FILE.exists() else "w"
    with open(ATTRIB_FILE, mode) as f:
        if mode == "w":
            f.write(
                "# Test image attribution\n"
                "# Photos sourced from iNaturalist under CC0 / CC-BY / CC-BY-SA licences.\n"
                "# This file is local-only (test_images/* is gitignored).\n\n"
            )
        else:
            f.write("\n# --- additional fetch ---\n\n")
        for a in attribs:
            f.write(a + "\n")
    print(f"\nSaved {saved_total} new photos to {OUT_DIR}")
    print(f"Attribution log: {ATTRIB_FILE}")
    if saved_total < 15:
        print("WARNING: fewer than 15 photos saved — validation set may be too thin.")
        sys.exit(1)


if __name__ == "__main__":
    main()
