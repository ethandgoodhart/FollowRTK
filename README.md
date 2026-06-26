# FollowRTK

Live lane annotation tool for FollowRTK routes.

## Local Development

```bash
npm ci
cp .env.example .env
npm start
```

Open http://localhost:3000.

At minimum, `.env` needs:

```bash
MAPBOX_TOKEN=your_mapbox_public_token
```

Without Supabase env vars, the app stores route data in local `annotations.json`.

## Supabase Setup

1. Create a Supabase project.
2. Open the Supabase SQL editor.
3. Run [supabase/schema.sql](supabase/schema.sql).
4. In Project Settings, copy:
   - Project URL
   - anon/public key
   - service role key
5. Add these to `.env` locally:

```bash
SUPABASE_URL=https://your-project-ref.supabase.co
SUPABASE_ANON_KEY=your_supabase_anon_or_publishable_key
SUPABASE_SERVICE_ROLE_KEY=your_supabase_service_role_key
ROUTE_DOCUMENT_ID=default
```

`SUPABASE_SERVICE_ROLE_KEY` is server-only. Do not paste it into frontend code.

The app stores one shared route document in `public.route_documents`. Friends load the same document, save edits through `/api/save`, and receive live reloads through Supabase Realtime when another browser updates the document.

## Vercel Deployment

1. Push this repo to GitHub.
2. Import the repo in Vercel.
3. Add these Vercel environment variables:

```bash
MAPBOX_TOKEN=your_mapbox_public_token
SUPABASE_URL=https://your-project-ref.supabase.co
SUPABASE_ANON_KEY=your_supabase_anon_or_publishable_key
SUPABASE_SERVICE_ROLE_KEY=your_supabase_service_role_key
ROUTE_DOCUMENT_ID=default
```

4. Deploy.

Vercel serves `public/` as the website and `api/` as serverless API routes.

## Collaboration Model

This first live version uses shared-document sync:

- New lines, connectors, erasers, dragged points, deletes, and undo actions auto-save.
- Supabase stores the latest route document.
- Other open browsers receive a realtime database event and reload the latest route document.
- Simultaneous edits are last-write-wins.

A later multi-user editing model can split each lane/connector/eraser into separate database rows to reduce overwrite conflicts.
