const MAPBOX_GEOCODE_URL = 'https://api.mapbox.com/geocoding/v5/mapbox.places';

const IGNORED_LABELS = new Set([
  'stanford',
  'california',
  'united states',
  'santa clara county'
]);

function isGenericLaneName(name = '') {
  return /^(road|lane|line|route|boundary)\s*\d*$/i.test(name.trim())
    || /^untitled/i.test(name.trim());
}

function sampleLinePoints(points, maxSamples = 5) {
  if (!Array.isArray(points) || points.length === 0) return [];
  if (points.length <= maxSamples) return points;

  const samples = [];
  for (let i = 0; i < maxSamples; i++) {
    const idx = Math.round((i * (points.length - 1)) / (maxSamples - 1));
    samples.push(points[idx]);
  }
  return samples;
}

function cleanLabel(label = '') {
  return label
    .replace(/^\d+\s+/, '')
    .replace(/\s+/g, ' ')
    .trim();
}

function labelFromFeature(feature) {
  if (!feature) return '';
  const type = feature.place_type?.[0] || '';
  if (type === 'address') return cleanLabel(feature.text);
  if (type === 'poi') return cleanLabel(feature.text);
  return '';
}

async function reverseGeocodePoint(point, token) {
  if (!token || !point?.lat || !point?.lng) return [];

  const params = new URLSearchParams({
    types: 'address,poi,neighborhood,locality,place',
    access_token: token
  });
  const url = `${MAPBOX_GEOCODE_URL}/${point.lng},${point.lat}.json?${params}`;
  const response = await fetch(url);
  if (!response.ok) return [];
  const data = await response.json();
  return Array.isArray(data.features) ? data.features : [];
}

async function suggestLabelForLine(line, token) {
  const samples = sampleLinePoints(line.points);
  const votes = new Map();
  const evidence = [];

  for (const point of samples) {
    const features = await reverseGeocodePoint(point, token);
    const labels = features
      .map(labelFromFeature)
      .filter(Boolean)
      .filter(label => !IGNORED_LABELS.has(label.toLowerCase()));

    if (labels.length === 0) continue;
    const label = labels[0];
    votes.set(label, (votes.get(label) || 0) + 1);
    evidence.push(label);
  }

  const ranked = [...votes.entries()].sort((a, b) => b[1] - a[1]);
  if (ranked.length === 0) {
    return {
      id: line.id,
      currentName: line.name,
      suggestedName: '',
      confidence: 0,
      votes: [],
      shouldApply: false
    };
  }

  const [suggestedName, count] = ranked[0];
  const confidence = samples.length > 0 ? count / samples.length : 0;
  return {
    id: line.id,
    currentName: line.name,
    suggestedName,
    confidence,
    votes: ranked.map(([name, voteCount]) => ({ name, count: voteCount })),
    evidence,
    shouldApply: isGenericLaneName(line.name) && count >= Math.min(2, samples.length)
  };
}

async function suggestLabelsForDocument(document, token) {
  const annotations = Array.isArray(document?.annotations) ? document.annotations : [];
  const suggestions = [];

  for (const annotation of annotations) {
    if (annotation.type !== 'lane' || !Array.isArray(annotation.points) || annotation.points.length < 2) {
      continue;
    }
    suggestions.push(await suggestLabelForLine(annotation, token));
  }

  return suggestions;
}

module.exports = {
  isGenericLaneName,
  suggestLabelsForDocument
};
