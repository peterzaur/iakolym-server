iakolym-server
==============

node js backend for iakolym

To run the server, make sure you have run `npm install` and type

`node app.js`

Then if you want to submit a request, send a JSON post request using something like postman to the following url:

`http://<host>:3000/api/steps`

The request payload would look something like {"step": "test"}

To check out the result, go into your favourite CLI and use sqlite3 to check out what's been added to your db!
