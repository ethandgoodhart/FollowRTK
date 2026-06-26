const fs = require('fs');
const path = require('path');
const { createClient } = require('@supabase/supabase-js');

const DATA_FILE = path.join(process.cwd(), 'annotations.json');
const ROUTE_DOCUMENT_ID = process.env.ROUTE_DOCUMENT_ID || 'default';

let supabaseAdmin = null;

function getSupabaseAdmin() {
  if (!process.env.SUPABASE_URL || !process.env.SUPABASE_SERVICE_ROLE_KEY) {
    return null;
  }

  if (!supabaseAdmin) {
    supabaseAdmin = createClient(
      process.env.SUPABASE_URL,
      process.env.SUPABASE_SERVICE_ROLE_KEY,
      {
        auth: {
          persistSession: false,
          autoRefreshToken: false
        }
      }
    );
  }

  return supabaseAdmin;
}

function emptyPayload() {
  return {
    annotations: [],
    connectors: [],
    manualCenterLines: [],
    eraserPoints: [],
    metadata: {
      campus: 'Stanford University',
      project: 'FollowRTK Self-Driving Golf Cart'
    }
  };
}

function normalizePayload(data) {
  return {
    ...emptyPayload(),
    ...data,
    annotations: Array.isArray(data?.annotations) ? data.annotations : [],
    connectors: Array.isArray(data?.connectors) ? data.connectors : [],
    manualCenterLines: Array.isArray(data?.manualCenterLines) ? data.manualCenterLines : [],
    eraserPoints: Array.isArray(data?.eraserPoints) ? data.eraserPoints : [],
    metadata: {
      ...emptyPayload().metadata,
      ...(data?.metadata || {}),
      savedAt: new Date().toISOString()
    }
  };
}

async function loadRouteDocument() {
  const supabase = getSupabaseAdmin();

  if (supabase) {
    const { data, error } = await supabase
      .from('route_documents')
      .select('payload')
      .eq('id', ROUTE_DOCUMENT_ID)
      .maybeSingle();

    if (error) throw error;
    if (data?.payload) return normalizePayload(data.payload);

    const payload = emptyPayload();
    const { error: insertError } = await supabase
      .from('route_documents')
      .upsert({
        id: ROUTE_DOCUMENT_ID,
        payload,
        updated_at: new Date().toISOString()
      }, { onConflict: 'id' });

    if (insertError) throw insertError;
    return payload;
  }

  if (!fs.existsSync(DATA_FILE)) return emptyPayload();
  return normalizePayload(JSON.parse(fs.readFileSync(DATA_FILE, 'utf-8')));
}

async function saveRouteDocument(data, updatedBy) {
  const payload = normalizePayload(data);
  const supabase = getSupabaseAdmin();

  if (supabase) {
    const { error } = await supabase
      .from('route_documents')
      .upsert({
        id: ROUTE_DOCUMENT_ID,
        payload,
        updated_by: updatedBy || null,
        updated_at: new Date().toISOString()
      }, { onConflict: 'id' });

    if (error) throw error;
  } else {
    fs.writeFileSync(DATA_FILE, JSON.stringify(payload, null, 2));
  }

  return {
    success: true,
    count: payload.annotations.length,
    connectors: payload.connectors.length,
    manualCenterLines: payload.manualCenterLines.length,
    eraserPoints: payload.eraserPoints.length,
    storage: supabase ? 'supabase' : 'local-json'
  };
}

function getPublicConfig() {
  const passwordRequired = !!process.env.APP_PASSWORD;
  return {
    mapboxToken: process.env.MAPBOX_TOKEN || '',
    supabaseUrl: process.env.SUPABASE_URL || '',
    supabaseAnonKey: process.env.SUPABASE_ANON_KEY || '',
    gpsWsUrl: process.env.GPS_WS_URL || '',
    routeDocumentId: ROUTE_DOCUMENT_ID,
    storageMode: getSupabaseAdmin() ? 'supabase' : 'local-json',
    passwordRequired
  };
}

function isAuthorized(req) {
  const password = process.env.APP_PASSWORD;
  if (!password) return true;
  return req.headers['x-followrtk-password'] === password;
}

module.exports = {
  getPublicConfig,
  isAuthorized,
  loadRouteDocument,
  saveRouteDocument
};
