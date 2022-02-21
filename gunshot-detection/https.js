var homeDir = require('path').join(require('os').homedir());
require('greenlock-express').create({
  version: 'draft-11'
, server: 'https://acme-v02.api.letsencrypt.org/directory'
//, server: 'https://acme-staging-v02.api.letsencrypt.org/directory'  // staging
, email: 'tejaswi@asu.edu'                                     // CHANGE THIS
, agreeTos: true
, approveDomains: [ 'costarica.acousticecologylab.org', 'www.costarica.acousticecologylab.org' ]              // CHANGE THIS
, store: require('greenlock-store-fs')
, configDir: homeDir
, app: require('./http.js')
//, communityMember: true
}).listen(8080, 8443);
