var express = require('express');
var router = express.Router();
var Step = require('../models/Step.js');

/* GET home page. */
router.get('/', function(req, res, next) {
  res.render('index', { title: 'Express' });
});

/* GET Steplist page. */
router.get('/steplist', function(req, res) {
  Step.find(function (err, steps) {
    if (err) return next(err);
    res.render('steplist', {
      "steplist": steps
    });
  });
});

module.exports = router;
