const { isAuthorized, saveRouteDocument } = require('../lib/storage');

module.exports = async function handler(req, res) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', 'POST');
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!isAuthorized(req)) return res.status(401).json({ error: 'Invalid password' });

  try {
    const result = await saveRouteDocument(req.body, req.headers['x-followrtk-client-id']);
    res.status(200).json(result);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
};
