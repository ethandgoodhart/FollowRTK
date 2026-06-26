const { isAuthorized, loadRouteDocument } = require('../lib/storage');

module.exports = async function handler(req, res) {
  if (!isAuthorized(req)) return res.status(401).json({ error: 'Invalid password' });

  try {
    const data = await loadRouteDocument();
    res.status(200).json(data);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
};
