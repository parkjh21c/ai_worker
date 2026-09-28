"""Export a run as a portable HTML file. Standard library only; no server needed."""
import argparse
import base64
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def collect_files(run_dir):
    """Only include dashboard records and observation JPEGs, never session prompts."""
    paths = [run_dir / name for name in ('run.json', 'host_result.json', 'result.json',
                                         'public/history.jsonl')]
    for pattern in ('rollout/*.json', 'calls/*_request.json', 'public/observations/*/observation.json',
                    'public/observations/*/*.jpg'):
        paths.extend(sorted(run_dir.glob(pattern)))
    files = {}
    for path in paths:
        if not path.is_file():
            continue
        key = path.relative_to(run_dir).as_posix()
        if path.suffix == '.jpg':
            files[key] = 'data:image/jpeg;base64,' + base64.b64encode(path.read_bytes()).decode('ascii')
        else:
            files[key] = path.read_text(encoding='utf-8')
    if 'run.json' not in files:
        raise ValueError(f'run.json not found in {run_dir}')
    return files


def export(run_dir, output):
    files = collect_files(Path(run_dir).resolve())
    # Escape HTML script terminators in arbitrary model/task text.
    payload = json.dumps(files, ensure_ascii=False).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    template = (HERE / 'viewer' / 'template.html').read_text(encoding='utf-8')
    live_script = (HERE / 'viewer' / 'live.js').read_text(encoding='utf-8')
    html = template.replace('__LIVE_SCRIPT__', live_script).replace('__RUN_FILES__', payload)
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding='utf-8')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path, default=HERE / 'run_viewer.html')
    args = parser.parse_args()
    try:
        output = export(args.run_dir, args.output)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'Export failed: {exc}\n')
    print(f'Open in your browser: {output}')


if __name__ == '__main__':
    main()
