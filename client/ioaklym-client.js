Step = new Mongo.Collection("steps2");

if (Meteor.isClient) {
  Template.body.helpers({
    steps: function () { 
      return Step.find({});
    }
  });
  console.log(Step.find({}));
  console.log("hi");
}

if (Meteor.isServer) {
  Meteor.startup(function () {
    console.log(Step.find({}));
    // code to run on server at startup
  });
}
