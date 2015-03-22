var mongoose = require('mongoose');

var StepSchema = new mongoose.Schema({
  user: String,
  angle: String,
  stride: String,
  swing: String,
  stance: String,
  timestamp: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Steps', StepSchema);
