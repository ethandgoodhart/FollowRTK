const express = require('express');
const fs = require('fs');
const path = require('path');

function loadEnv() {
  const envPath = path.join(__dirname, '.env');
  if (!fs.existsSync(envPath)) return;
  for (const line of fs.readFileSync(envPath, 'utf-8').split('\n')) {
    const match = line.match(/^([^#=]+)=(.*)$/);
    if (match && !process.env[match[1].trim()]) {
      process.env[match[1].trim()] = match[2].trim();
    }
  }
}

loadEnv();

const { getPublicConfig, isAuthorized, loadRouteDocument, saveRouteDocument } = require('./lib/storage');

const app = express();
const PORT = 3000;

app.use(express.json({ limit: '10mb' }));
app.use(express.static(path.join(__dirname, 'public')));

app.get('/api/config', (req, res) => {
  res.json(getPublicConfig());
});

app.get('/api/load', async (req, res) => {
  if (!isAuthorized(req)) return res.status(401).json({ error: 'Invalid password' });

  try {
    const data = await loadRouteDocument();
    res.json(data);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

app.post('/api/save', async (req, res) => {
  if (!isAuthorized(req)) return res.status(401).json({ error: 'Invalid password' });

  const data = req.body;
  if (!data || !Array.isArray(data.annotations)) {
    return res.status(400).json({ error: 'Invalid data format. Expected { annotations: [...], eraserPoints?: [...] }' });
  }

  try {
    const result = await saveRouteDocument(data, req.headers['x-followrtk-client-id']);
    res.json(result);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

app.listen(PORT, () => {
  console.log(`Lane annotator running at http://localhost:${PORT}`);
});
