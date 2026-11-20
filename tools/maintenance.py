#!/usr/bin/env python3
"""Workspace-scoped cache housekeeping. Original maps, photos, code and logs are never selected."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

def collect(root,older_than_days=7,now=None):
    root=Path(root).resolve()
    cutoff=(time.time() if now is None else now)-older_than_days*86400
    records=[]
    # Do not enter test/reference workspaces, symlinks or build/install outputs.
    import os
    for parent,dirs,files in os.walk(str(root),followlinks=False):
        dirs[:]=[d for d in dirs if d not in ('.git','build','devel','install','archives','vendor','reference','candidates')
                 and not (Path(parent)/d).is_symlink()]
        for name in files:
            path=Path(parent)/name
            if path.is_symlink() or path.suffix not in ('.pyc','.pyo'):continue
            resolved=path.resolve()
            resolved.relative_to(root)
            stat=path.stat()
            if stat.st_mtime<cutoff:
                records.append({'path':str(resolved),'size':stat.st_size,'mtime_ns':stat.st_mtime_ns})
    return records

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true')
    parser.add_argument('--older-than-days',type=int,default=7)
    args=parser.parse_args()
    if args.older_than_days<1:raise ValueError('minimum retention is one day')
    root=Path(__file__).resolve().parents[1]
    records=collect(root,args.older_than_days)
    deleted=[]
    if args.apply:
        for record in records:
            path=Path(record['path'])
            path.resolve().relative_to(root.resolve())
            if path.is_symlink() or path.stat().st_mtime_ns!=record['mtime_ns']:continue
            path.unlink()
            deleted.append(record['path'])
    print(json.dumps({'root':str(root),'retention_days':args.older_than_days,'mode':'apply' if args.apply else 'check',
        'candidates':records,'bytes':sum(r['size'] for r in records),'deleted':deleted,
        'time':datetime.now().astimezone().isoformat()},indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
