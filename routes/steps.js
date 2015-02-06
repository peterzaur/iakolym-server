var express = require('express');
var router = express.Router();

var mongoose = require('mongoose');
var Step = require('../models/Step.js');

/* GET /steps listing. */
router.get('/', function(req, res, next) {
  Step.find(function (err, steps) {
    if (err) return next(err);
    res.json(steps);
  });
});

/* GET /steps/id */
router.get('/:id', function(req, res, next) {
  Step.findById(req.params.id, function (err, post) {
    if (err) return next(err);
    res.json(post);
  });
});

/* POST /steps */
router.post('/', function(req, res, next) {
  Step.create(req.body, function (err, post) {
    if (err) return next(err);
    res.json(post);
  });
});

/* PUT /steps/:id */
router.put('/:id', function(req, res, next) {
  Step.findByIdAndUpdate(req.params.id, req.body, function (err, post) {
    if (err) return next(err);
    res.json(post);
  });
});

/* DELETE /steps/:id */
router.delete('/:id', function(req, res, next) {
  Step.findByIdAndRemove(req.params.id, req.body, function (err, post) {
    if (err) return next(err);
    res.json(post);
  });
});

module.exports = router;
