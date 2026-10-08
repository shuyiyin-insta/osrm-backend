import test from 'tape';
import { OSRM, data_path, mld_data_path, two_test_coordinates } from './constants.js';

test('isochrone: returns a GeoJSON FeatureCollection on MLD', (assert) => {
  assert.plan(6);
  const osrm = new OSRM({path: mld_data_path, algorithm: 'MLD'});
  osrm.isochrone({
    coordinates: [two_test_coordinates[0]],
    contours_seconds: [300, 600],
    polygons: false,
    generalize: 25,
    denoise: 0.1,
  }, (err, result) => {
    assert.ifError(err);
    assert.equal(result.type, 'FeatureCollection');
    assert.notOk(Object.hasOwn(result, 'weight_name'));
    assert.ok(Array.isArray(result.features));
    assert.equal(result.features.length, 2);
    assert.ok(result.features.every((feature) =>
      feature.type === 'Feature' && feature.geometry.type === 'MultiLineString' &&
      Array.isArray(feature.geometry.coordinates) &&
      Number.isFinite(feature.properties.effective_contour_seconds) &&
      feature.properties.effective_contour_seconds <= feature.properties.contour_seconds));
  });
});

test('isochrone: returns JSON in a buffer when requested', (assert) => {
  assert.plan(4);
  const osrm = new OSRM({path: mld_data_path, algorithm: 'MLD'});
  osrm.isochrone({
    coordinates: [two_test_coordinates[0]],
    contours_seconds: [300],
  }, {format: 'buffer'}, (err, result) => {
    assert.ifError(err);
    assert.ok(result instanceof Buffer);
    const response = JSON.parse(result);
    assert.equal(response.type, 'FeatureCollection');
    assert.equal(response.features.length, 1);
  });
});

test('isochrone: parses parameters and rejects invalid arguments', (assert) => {
  assert.plan(23);
  const osrm = new OSRM({path: mld_data_path, algorithm: 'MLD'});
  const options = { coordinates: [two_test_coordinates[0]] };

  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /contours_seconds must be a non-empty array of positive finite durations in seconds/);

  options.contours = [300];
  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /contours_seconds must be a non-empty array of positive finite durations in seconds/);

  options.contours_seconds = [];
  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /contours_seconds must be a non-empty array of positive finite durations in seconds/);

  options.contours_seconds = [0];
  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /contours_seconds must be a non-empty array of positive finite durations in seconds/);

  options.contours_seconds = [Infinity];
  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /contours_seconds must be a non-empty array of positive finite durations in seconds/);

  options.contours_seconds = [300];
  options.direction = 'sideways';
  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /Direction must be a string: \[outbound, inbound\]/);

  options.direction = 'outbound';
  options.polygons = 'true';
  assert.throws(() => { osrm.isochrone(options, () => {}); },
    /polygons must be of type Boolean/);

  options.polygons = false;
  assert.doesNotThrow(() => { osrm.isochrone(options, () => {}); });

  options.generalize = 0;
  assert.doesNotThrow(() => { osrm.isochrone(options, () => {}); });

  options.generalize = 25;
  assert.doesNotThrow(() => { osrm.isochrone(options, () => {}); });

  for (const invalidGeneralize of [-1, Infinity, NaN, '25']) {
    options.generalize = invalidGeneralize;
    assert.throws(() => { osrm.isochrone(options, () => {}); },
      /Generalize must be a finite nonnegative tolerance in metres/);
  }

  delete options.generalize;
  for (const denoise of [0, 0.25, 1]) {
    options.denoise = denoise;
    assert.doesNotThrow(() => { osrm.isochrone(options, () => {}); });
  }

  for (const invalidDenoise of [-0.1, 1.1, Infinity, NaN, '0.25']) {
    options.denoise = invalidDenoise;
    assert.throws(() => { osrm.isochrone(options, () => {}); },
      /Denoise must be a finite number between zero and one/);
  }

  assert.throws(() => {
    osrm.isochrone({coordinates: two_test_coordinates, contours_seconds: [300]}, () => {});
  }, /Exactly one coordinate pair must be provided/);
});

test('isochrone: rejects flatbuffers output asynchronously', (assert) => {
  assert.plan(1);
  const osrm = new OSRM({path: mld_data_path, algorithm: 'MLD'});

  osrm.isochrone({
    coordinates: [two_test_coordinates[0]],
    contours_seconds: [300],
    format: 'flatbuffers',
  }, (err) => {
    assert.equal(err.message, 'The isochrone service only supports JSON/GeoJSON output.');
  });
});

test('isochrone: validates its search-node limit independently of other limits', (assert) => {
  const invalidLimits = [0, -2, -2147483648, 2147483648, 1.5, Infinity, NaN, '1', true, null];
  assert.plan(invalidLimits.length + 3);
  for (const max_isochrone_search_nodes of invalidLimits) {
    assert.throws(() => {
      new OSRM({path: data_path, max_isochrone_search_nodes});
    }, /max_isochrone_search_nodes must be -1 or a positive integral number within int range/);
  }
  for (const max_isochrone_search_nodes of [-1, 1, 2147483647]) {
    assert.ok(new OSRM({path: data_path, max_isochrone_search_nodes, max_locations_trip: 1}));
  }
});

for (const [algorithm, path] of [['CH', data_path], ['MLD', mld_data_path]]) {
  test(`isochrone: ${algorithm} search limit does not affect route or table`, (assert) => {
    assert.plan(6);
    const osrm = new OSRM({path, algorithm, max_isochrone_search_nodes: 1});
    const options = {coordinates: [two_test_coordinates[0]], contours_seconds: [300]};
    osrm.isochrone(options, (err) => {
      assert.equal(err?.message, 'TooBig');
      osrm.route({coordinates: two_test_coordinates}, (routeErr, route) => {
        assert.ifError(routeErr);
        assert.ok(route.routes.length > 0);
        osrm.table({coordinates: two_test_coordinates}, (tableErr, table) => {
          assert.ifError(tableErr);
          assert.equal(table.durations.length, 2);
          osrm.isochrone(options, (repeatErr) => {
            assert.equal(repeatErr?.message, 'TooBig');
          });
        });
      });
    });
  });

  for (const max_isochrone_search_nodes of [-1, 10000000]) {
    test(`isochrone: ${algorithm} accepts search limit ${max_isochrone_search_nodes}`, (assert) => {
      assert.plan(2);
      const osrm = new OSRM({path, algorithm, max_isochrone_search_nodes});
      osrm.isochrone({coordinates: [two_test_coordinates[0]], contours_seconds: [300]}, (err, result) => {
        assert.ifError(err);
        assert.equal(result.features.length, 1);
      });
    });
  }
}
