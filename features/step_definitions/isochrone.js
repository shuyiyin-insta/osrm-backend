import assert from 'node:assert';

import { Then, When } from '@cucumber/cucumber';

function parseContourSeconds(contoursSeconds) {
  const values = contoursSeconds.split(',').map(Number);
  values.forEach((value) => {
    assert.ok(
      Number.isFinite(value),
      `invalid contours_seconds value in test: ${contoursSeconds}`,
    );
  });
  return values;
}

async function requestIsochrone(
  world,
  nodeName,
  contoursSeconds,
  options = {},
) {
  const node = world.findNodeByName(nodeName);
  assert.ok(node, `unknown isochrone input node "${nodeName}"`);

  const parameters = Object.assign({}, world.queryParams, options, {
    coordinates: [[node.lon, node.lat].join(',')],
    contours_seconds: contoursSeconds,
  });
  const { response, body } = await new Promise((resolve, reject) => {
    world.requestPath('isochrone', parameters, (err, response, body) => {
      if (err) return reject(err);
      resolve({ response, body });
    });
  });

  let json;
  try {
    json = JSON.parse(body);
  } catch {
    throw new Error(`isochrone response is not JSON: ${body}`);
  }
  return { response, json, body };
}

function assertPosition(position) {
  assert.ok(
    Array.isArray(position) && position.length === 2,
    'expected a GeoJSON position',
  );
  assert.ok(Number.isFinite(position[0]), 'expected finite longitude');
  assert.ok(Number.isFinite(position[1]), 'expected finite latitude');
  assert.ok(
    position[0] >= -180 && position[0] <= 180,
    'longitude is outside [-180, 180]',
  );
  assert.ok(
    position[1] >= -90 && position[1] <= 90,
    'latitude is outside [-90, 90]',
  );
}

function assertRing(ring) {
  assert.ok(
    Array.isArray(ring) && ring.length >= 4,
    'expected a non-empty linear ring',
  );
  ring.forEach(assertPosition);
  assert.deepStrictEqual(
    ring[0],
    ring[ring.length - 1],
    'expected a closed linear ring',
  );
}

function assertGeometry(geometry, geometryType) {
  assert.strictEqual(geometry.type, geometryType);
  assert.ok(
    Array.isArray(geometry.coordinates),
    'expected geometry coordinates',
  );
  assert.ok(
    geometry.coordinates.length > 0,
    'expected non-empty geometry coordinates',
  );

  if (geometryType === 'MultiPolygon') {
    geometry.coordinates.forEach((polygon) => {
      assert.ok(
        Array.isArray(polygon) && polygon.length > 0,
        'expected a polygon with a ring',
      );
      polygon.forEach(assertRing);
    });
  } else {
    geometry.coordinates.forEach(assertRing);
  }
}

function pointIsOnSegment(point, start, end) {
  const [x, y] = point;
  const [startX, startY] = start;
  const [endX, endY] = end;
  const squaredLength = (endX - startX) ** 2 + (endY - startY) ** 2;
  if (squaredLength <= 1e-24)
    return Math.abs(x - startX) <= 1e-12 && Math.abs(y - startY) <= 1e-12;

  const crossProduct =
    (x - startX) * (endY - startY) - (y - startY) * (endX - startX);
  if (Math.abs(crossProduct) > 1e-12) return false;

  const dotProduct =
    (x - startX) * (endX - startX) + (y - startY) * (endY - startY);
  return dotProduct >= 0 && dotProduct <= squaredLength;
}

function pointIsInRing(point, ring) {
  let inside = false;
  for (
    let current = 0, previous = ring.length - 1;
    current < ring.length;
    previous = current++
  ) {
    const currentPoint = ring[current];
    const previousPoint = ring[previous];
    if (pointIsOnSegment(point, previousPoint, currentPoint)) return true;

    const crossesLatitude =
      currentPoint[1] > point[1] !== previousPoint[1] > point[1];
    const intersectionLongitude =
      ((previousPoint[0] - currentPoint[0]) * (point[1] - currentPoint[1])) /
        (previousPoint[1] - currentPoint[1]) +
      currentPoint[0];
    if (crossesLatitude && point[0] < intersectionLongitude) inside = !inside;
  }
  return inside;
}

function geometryContainsPoint(geometry, point) {
  return geometry.coordinates.some((polygon) => {
    const [outer, ...holes] = polygon;
    return (
      pointIsInRing(point, outer) &&
      !holes.some((hole) => pointIsInRing(point, hole))
    );
  });
}

function maximumLongitude(geometry) {
  return Math.max(
    ...geometry.coordinates.flatMap((polygon) =>
      polygon.flatMap((ring) => ring.map((position) => position[0])),
    ),
  );
}

function assertFeatureCollection(result, featureCount, geometryType) {
  assert.strictEqual(
    result.response.statusCode,
    200,
    `unexpected response: ${result.body}`,
  );
  assert.ok(
    (result.response.headers['content-type'] || '').includes('json'),
    `isochrone must be served as JSON, got ${result.response.headers['content-type']}`,
  );
  assert.strictEqual(
    result.json.code,
    'Ok',
    `unexpected response: ${result.body}`,
  );
  assert.strictEqual(result.json.type, 'FeatureCollection');
  assert.ok(Array.isArray(result.json.features), 'expected GeoJSON features');
  assert.strictEqual(result.json.features.length, featureCount);

  result.json.features.forEach((feature) => {
    assert.strictEqual(feature.type, 'Feature');
    assert.ok(feature.properties, 'expected feature properties');
    assert.ok(
      Number.isFinite(feature.properties.contour_seconds),
      'expected numeric contour_seconds property',
    );
    assert.ok(
      Number.isFinite(feature.properties.effective_contour_seconds),
      'expected numeric effective_contour_seconds property',
    );
    assert.ok(
      feature.properties.effective_contour_seconds <=
        feature.properties.contour_seconds,
      'effective_contour_seconds must not exceed the requested contours_seconds value',
    );
    assertGeometry(feature.geometry, geometryType);
  });
}

When(
  /^I request an isochrone from "([a-z0-9])" with contours_seconds "([^"]+)"$/,
  async function (node, contoursSeconds) {
    await this.reprocessAndLoadData();
    this.isochroneResponse = await requestIsochrone(
      this,
      node,
      contoursSeconds,
    );
  },
);

When(
  /^I request an isochrone from "([a-z0-9])" with contours_seconds "([^"]+)" and polygons "([^"]+)"$/,
  async function (node, contoursSeconds, polygons) {
    await this.reprocessAndLoadData();
    this.isochroneResponse = await requestIsochrone(
      this,
      node,
      contoursSeconds,
      { polygons },
    );
  },
);

When(
  /^I request equivalent raw and zero-processed isochrones from "([a-z0-9])" with contours_seconds "([^"]+)"$/,
  async function (node, contoursSeconds) {
    await this.reprocessAndLoadData();
    this.isochroneResponse = await requestIsochrone(
      this,
      node,
      contoursSeconds,
    );
    this.zeroProcessedIsochroneResponse = await requestIsochrone(
      this,
      node,
      contoursSeconds,
      {
        denoise: '0',
        generalize: '0',
      },
    );
  },
);

When(
  /^I request inbound and outbound isochrones from "([a-z0-9])" with contours_seconds "([^"]+)"$/,
  async function (node, contoursSeconds) {
    await this.reprocessAndLoadData();
    this.outboundIsochroneResponse = await requestIsochrone(
      this,
      node,
      contoursSeconds,
      {
        direction: 'outbound',
      },
    );
    this.inboundIsochroneResponse = await requestIsochrone(
      this,
      node,
      contoursSeconds,
      {
        direction: 'inbound',
      },
    );
  },
);

Then(
  /^the isochrone response should be a GeoJSON FeatureCollection with "(\d+)" "(MultiPolygon|MultiLineString)" features$/,
  function (featureCount, geometryType) {
    assert.ok(this.isochroneResponse, 'no isochrone response was recorded');
    assertFeatureCollection(
      this.isochroneResponse,
      Number(featureCount),
      geometryType,
    );
  },
);

Then(
  /^the isochrone contour_seconds properties should be "([^"]+)"$/,
  function (contoursSeconds) {
    assert.ok(this.isochroneResponse, 'no isochrone response was recorded');
    assert.deepStrictEqual(
      this.isochroneResponse.json.features.map(
        (feature) => feature.properties.contour_seconds,
      ),
      parseContourSeconds(contoursSeconds),
    );
  },
);

Then(
  /^the isochrone response should have "(\d+)" waypoint$/,
  function (waypointCount) {
    assert.ok(this.isochroneResponse, 'no isochrone response was recorded');
    assert.ok(
      Array.isArray(this.isochroneResponse.json.waypoints),
      'expected waypoints',
    );
    assert.strictEqual(
      this.isochroneResponse.json.waypoints.length,
      Number(waypointCount),
    );
  },
);

Then(
  /^the raw and zero-processed isochrone responses should be identical$/,
  function () {
    assert.ok(this.isochroneResponse, 'no raw isochrone response was recorded');
    assert.ok(
      this.zeroProcessedIsochroneResponse,
      'no zero-processed isochrone response was recorded',
    );
    assert.deepStrictEqual(
      this.zeroProcessedIsochroneResponse.json,
      this.isochroneResponse.json,
    );
  },
);

Then(
  /^the inbound and outbound isochrone geometries should differ$/,
  function () {
    assert.ok(
      this.outboundIsochroneResponse,
      'no outbound isochrone response was recorded',
    );
    assert.ok(
      this.inboundIsochroneResponse,
      'no inbound isochrone response was recorded',
    );
    assertFeatureCollection(this.outboundIsochroneResponse, 1, 'MultiPolygon');
    assertFeatureCollection(this.inboundIsochroneResponse, 1, 'MultiPolygon');
    assert.notDeepStrictEqual(
      this.outboundIsochroneResponse.json.features.map(
        (feature) => feature.geometry,
      ),
      this.inboundIsochroneResponse.json.features.map(
        (feature) => feature.geometry,
      ),
    );
  },
);

Then(
  /^the isochrone should contain "([a-z0-9])" and not contain "([a-z0-9])"$/,
  function (includedName, excludedName) {
    assert.ok(this.isochroneResponse, 'no isochrone response was recorded');
    const included = this.findNodeByName(includedName);
    const excluded = this.findNodeByName(excludedName);
    assert.ok(included, `unknown expected node "${includedName}"`);
    assert.ok(excluded, `unknown excluded node "${excludedName}"`);

    const geometry = this.isochroneResponse.json.features[0].geometry;
    assert.ok(
      geometryContainsPoint(geometry, [included.lon, included.lat]),
      `expected isochrone to contain ${includedName}`,
    );
    assert.ok(
      !geometryContainsPoint(geometry, [excluded.lon, excluded.lat]),
      `expected isochrone not to contain ${excludedName}`,
    );
  },
);

Then(
  /^the isochrone boundary should stop partway from "([a-z0-9])" to "([a-z0-9])"$/,
  function (sourceName, targetName) {
    assert.ok(this.isochroneResponse, 'no isochrone response was recorded');
    const source = this.findNodeByName(sourceName);
    const target = this.findNodeByName(targetName);
    assert.ok(source, `unknown expected source node "${sourceName}"`);
    assert.ok(target, `unknown expected target node "${targetName}"`);
    assert.ok(
      target.lon > source.lon,
      'interpolation acceptance test requires eastbound edge',
    );

    const geometry = this.isochroneResponse.json.features[0].geometry;
    const boundaryLongitude = maximumLongitude(geometry);
    const oneQuarter = source.lon + (target.lon - source.lon) / 4;
    const threeQuarters = source.lon + ((target.lon - source.lon) * 3) / 4;
    assert.ok(
      boundaryLongitude > oneQuarter,
      `expected cutoff geometry to progress past ${oneQuarter}, got ${boundaryLongitude}`,
    );
    assert.ok(
      boundaryLongitude < threeQuarters,
      `expected cutoff geometry to stop before ${threeQuarters}, got ${boundaryLongitude}`,
    );
  },
);

Then(/^the isochrone HTTP status should be (\d+)$/, function (status) {
  assert.strictEqual(
    this.response.statusCode,
    Number(status),
    `unexpected response: ${this.body}`,
  );
});
