var mongoose = require('mongoose');

var StepSchema = new mongoose.Schema({
  user: String,
  angle: Number,
  timestamp: { type: Date, default: Date.now },
});

module.exports = mongoose.model('Steps', StepSchema);
