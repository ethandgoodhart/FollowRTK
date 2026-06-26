const { getPublicConfig } = require('../lib/storage');

module.exports = function handler(req, res) {
  res.status(200).json(getPublicConfig());
};
