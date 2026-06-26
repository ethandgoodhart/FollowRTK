const { suggestLabelsForDocument } = require('../lib/autoLabel');
const { isAuthorized } = require('../lib/storage');

module.exports = async function handler(req, res) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', 'POST');
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!isAuthorized(req)) return res.status(401).json({ error: 'Invalid password' });
  if (!process.env.MAPBOX_TOKEN) return res.status(500).json({ error: 'MAPBOX_TOKEN is not configured' });

  try {
    const suggestions = await suggestLabelsForDocument(req.body, process.env.MAPBOX_TOKEN);
    res.status(200).json({ suggestions });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
};
