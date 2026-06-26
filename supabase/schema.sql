create table if not exists public.route_documents (
  id text primary key,
  payload jsonb not null default '{"annotations":[],"connectors":[],"manualCenterLines":[],"suppressedAutoCenterLineIds":[],"eraserPoints":[]}'::jsonb,
  updated_by text,
  updated_at timestamptz not null default now()
);

alter table public.route_documents enable row level security;

drop policy if exists "route documents are readable" on public.route_documents;
create policy "route documents are readable"
  on public.route_documents
  for select
  to anon
  using (true);

grant select on public.route_documents to anon;

insert into public.route_documents (id, payload)
values (
  'default',
  '{"annotations":[],"connectors":[],"manualCenterLines":[],"suppressedAutoCenterLineIds":[],"eraserPoints":[],"metadata":{"campus":"Stanford University","project":"FollowRTK Self-Driving Golf Cart"}}'::jsonb
)
on conflict (id) do nothing;

do $$
begin
  alter publication supabase_realtime add table public.route_documents;
exception
  when duplicate_object then null;
end $$;
